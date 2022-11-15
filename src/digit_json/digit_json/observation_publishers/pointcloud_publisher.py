import asyncio
import json
import select
import socket
import sys
from enum import Enum
from time import perf_counter

# import cv2
import numpy as np
import websockets

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField

# Point Cloud Publisher
class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('PointCloud2_pub_node')
        self.publisher_ = self.create_publisher(
            PointCloud2, 'upper_velodyne_vlp16/depth/points', 10)

# Request perception stream start by name
async def start_stream(name):
    # Request perception stream and wait for response
    async with websockets.connect('ws://10.10.1.1:8080',
                                  subprotocols=['json-v1-agility']) as ws:
        await ws.send(json.dumps(['perception-stream-start', {
            'stream': name,
            'flow-control': 'none'}]))
        msg = json.loads(await ws.recv())

    # Verify received message is the correct type
    if msg[0] != 'perception-stream-response':
        raise ValueError(f"must be perception-stream-response, not {msg[0]}")
    port = msg[1]['port']
    print(f"Successfully started {name} stream at port {port}")
    return port


# Attempts to find the JSON header message in the incoming byte stream
def find_json(data):
    text = data.decode('latin-1')
    start = text.find('["perception-stream-frame"')
    count = 0
    for i, ch in enumerate(text[start:]):
        if ch == '[':
            count += 1
        elif ch == ']':
            count -= 1
            if count == 0:
                end = start + i + 1
                return text[start:end], data[end:]
    return None, data


# Define a helper class for parsing the frame info
class FrameInfo:
    def __init__(self, json_msg):
        self.format = json_msg[1].get('format')
        self.size = json_msg[1].get('size')
        self.height = json_msg[1].get('height')
        self.width = json_msg[1].get('width')

        if self.format == 'RGB8':
            self.channels = 3
            self.bit_depth = np.uint8
        elif self.format == 'Gray8':
            self.channels = 1
            self.bit_depth = np.uint8
        elif self.format == 'Depth16':
            self.channels = 1
            self.bit_depth = np.uint16
        elif self.format == 'XYZ':
            self.channels = 3
            self.bit_depth = np.float32
        elif self.format == 'XYZI':
            self.channels = 4
            self.bit_depth = np.float32
        else:
            raise ValueError("must be RGB8, Gray8, Depth16, XYZ or XYZI, not "
                             f"{self.format}")

        bytes_per_channel = int(np.dtype(self.bit_depth).itemsize)
        if self.format in ['RGB8', 'Gray8', 'Depth16']:
            self.size = self.height * self.width * self.channels * bytes_per_channel
        elif self.format in ['XYZ', 'XYZI']:
            self.size = self.size * self.channels * bytes_per_channel

        # transform from device to stream
        tf_list = json_msg[1].get('T-device-to-stream')
        self.tf_device_to_stream = np.transpose(np.array(tf_list))
        

# Array to PointField
def arr_to_fields(cloud):
    """Convert a numpy record datatype into a list of PointFields.
    """
    dtype_enum = {'int8': 1,
                  'uint8': 2,
                  'int16': 3,
                  'uint16': 4,
                  'int32': 5,
                  'uint32': 6,
                  'float32': 7,
                  'float64': 8}
    fields = []
    for field_name in cloud.dtype.names:
        np_field_type, field_offset = cloud.dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name

        pf.datatype = dtype_enum[str(np_field_type)]
        pf.offset = field_offset
        pf.count = 1  
        fields.append(pf)
    return fields

# Connect to perception stream, process incoming byte stream, and display images
def process_stream_and_publish(name, port, pub):
    # Establish socket connection
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('10.10.1.1', port))
    sock.setblocking(0)

    # Variables for parsing the data
    data = bytearray()
    ReadState = Enum('ReadState', ['FIND_JSON', 'READ_DATA', 'PROCESS'])
    state = ReadState.FIND_JSON
    read_chunk_size = 8192
    frame_chunk_size = 10
    frames_recieved = 20
    time_count = perf_counter()

    while rclpy.ok():
        # Check if we can read anything from the socket
        readable, writable, exceptional = select.select([sock], [sock], [], 1)

        if len(writable):
            # # Request another set of frames if the current set has been read
            if perf_counter() - time_count > 1:
                # print(f"Requesting {frames_recieved} frames")
                writable[0].send(
                    (frames_recieved+3).to_bytes(1, byteorder='big'))
                # print(frames_recieved)
                frames_recieved = 0
                time_count = perf_counter()

        if len(readable):
            # Append received data chunk to frame data
            data = data + readable[0].recv(read_chunk_size)

            # Search for the JSON header and advance to next state when found
            if state == ReadState.FIND_JSON:
                json_msg, data = find_json(data)
                if json_msg:
                    frame_info = FrameInfo(json.loads(json_msg))
                    state = ReadState.READ_DATA

            # Read the exact amount of data specified by frame size and advance
            # to next state when complete
            elif state == ReadState.READ_DATA:
                if frame_info.size <= len(data):
                    state = ReadState.PROCESS
                elif frame_info.size - len(data) < read_chunk_size:
                    read_chunk_size = frame_info.size - len(data)

            # Process the read frame data using the JSON header frame info
            elif state == ReadState.PROCESS:
                # Update parsing variables
                state = ReadState.FIND_JSON
                read_chunk_size = 8192
                frames_recieved += 1

                # Interpret buffer as 1-D array with specified frame datatype
                buffer = data[:frame_info.size]
                a = np.frombuffer(buffer, dtype=frame_info.bit_depth)

                # Reshape array with specified frame dimensions
                depth_points_stream_frame = a.reshape((int(a.size / frame_info.channels), frame_info.channels))  # dim = N x 3
                    
                # Rotate by Transformation device-to-stream (given by agility)
                ones_array = np.ones((int(a.size / frame_info.channels),1))
                depth_points_stream_frame_homogeneous = np.append(depth_points_stream_frame, ones_array, axis=1)  # dim = N x 4 (For multiplying by homogeneous transformations)
                T = frame_info.tf_device_to_stream
                depth_points_device_frame_homogenous = np.transpose( np.matmul(T, np.transpose(depth_points_stream_frame_homogeneous)) ) # dim = ( (4 x 4) x (4 x N) )^T
                depth_points_device_frame = np.delete(depth_points_device_frame_homogenous, -1, 1) # N x 3
                    
                # Append Intensity to cloud array
                cloud_array_intensity = np.ones((int(a.size / frame_info.channels), 1)) # N x 1
                depth_points_with_intensity_device_frame = np.append(depth_points_device_frame, cloud_array_intensity,1) # N x 4

                # convert to structured array
                dty = np.dtype(
                    [('x', 'float32'), ('y', 'float32'), ('z', 'float32'), ('intensity','float32')])
                depth_points = np.zeros(depth_points_with_intensity_device_frame.shape[0], dtype=dty)
                depth_points['x'] = depth_points_with_intensity_device_frame[:, 0]
                depth_points['y'] = depth_points_with_intensity_device_frame[:, 1]
                depth_points['z'] = depth_points_with_intensity_device_frame[:, 2]
                depth_points['intensity'] = depth_points_with_intensity_device_frame[:,3]
                depth_points = np.atleast_2d(depth_points)

                # Create and Update PointCloud2 message
                cloud_msg = PointCloud2()
                cloud_msg.header.stamp = pub.get_clock().now().to_msg()
                cloud_msg.header.frame_id = 'base_link'
                cloud_msg.height = depth_points.shape[0]
                cloud_msg.width = depth_points.shape[1]
                cloud_msg.fields = arr_to_fields(depth_points)
                cloud_msg.is_bigendian = False  # assumption
                cloud_msg.point_step = depth_points.dtype.itemsize
                cloud_msg.row_step = cloud_msg.point_step*depth_points.shape[1]
                cloud_msg.is_dense = all([np.isfinite(depth_points[fname]).all() for fname in depth_points.dtype.names])
                cloud_msg.data = depth_points.tobytes()  # why in bytes

                # Publish message
                pub.publisher_.publish(cloud_msg)
                
                if len(select.select([sock], [sock], [], 1)[0]):
                    # delete the rest of the buffer in the socket
                    data = bytearray(read_chunk_size)
                    size = readable[0].recv_into(data, read_chunk_size)
                # and socket exists (use select) why do you require this
                while size == read_chunk_size & len(select.select([sock], [sock], [], 1)[0]):
                    size = readable[0].recv_into(data, read_chunk_size)
                    pass
                data = data[:size]

# Main
def main(args=None):
    rclpy.init(args=args)
    pointcloud_publisher = PointCloudPublisher()
    velodyne = 'upper-velodyne-vlp16/depth/points'
    port = asyncio.get_event_loop().run_until_complete(start_stream(velodyne))
    process_stream_and_publish(velodyne, port, pointcloud_publisher)

if __name__ == '__main__':
    try:
        main()

    except websockets.exceptions.ConnectionClosedError:
        sys.exit("Connection to robot lost")
    except (ConnectionRefusedError, asyncio.exceptions.TimeoutError):
        sys.exit("Could not connect to robot")
    except KeyboardInterrupt:
        sys.exit("Caught user exit")
