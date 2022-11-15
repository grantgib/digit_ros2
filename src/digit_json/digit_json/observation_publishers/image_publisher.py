import asyncio
import json
import select
import socket
import sys
from enum import Enum
from time import perf_counter

import numpy as np
import websockets
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField

# Publisher
class ImagePublisher(Node):
    def __init__(self,node_name,topic_name):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(
            Image, topic_name, 10)

# Request perception stream start by name
async def start_stream(name):
    # Request perception stream and wait for response
    async with websockets.connect('ws://10.10.1.1:8080',
                                  subprotocols=['json-v1-agility']) as ws:
        await ws.send(json.dumps(['perception-stream-start', {
            'stream': name,
            'flow-control': 'framerate'}]))
        msg = json.loads(await ws.recv())

    # Verify received message is the correct type
    if msg[0] != 'perception-stream-response':
        print(msg)
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

# Array to PointField
def arr_to_fields(points):
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
    for field_name in points.dtype.names:
        np_field_type, field_offset = points.dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name

        pf.datatype = dtype_enum[str(np_field_type)]
        pf.offset = field_offset
        pf.count = 1  # is this ever more than one?
        fields.append(pf)
    return fields

# Connect to perception stream, process incoming byte stream, and display images
def process_stream_and_publish(name, port, image_pub, bridge, frameid):
    # Establish socket connection
    # print(f"Establishing connection to {name} stream at port {port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('10.10.1.1', port))
    sock.setblocking(0)

    # Variables for parsing the data
    data = bytearray()
    ReadState = Enum('ReadState', ['FIND_JSON', 'READ_DATA', 'PROCESS'])
    state = ReadState.FIND_JSON
    read_chunk_size = 8192
    frame_chunk_size = 10
    # frames_remaining = 0
    frames_recieved = 15
    time_count = perf_counter()

    while rclpy.ok():
        # Check if we can read anything from the socket
        readable, writable, exceptional = select.select([sock], [sock], [], 1)

        if len(writable):
            # # Request another set of frames if the current set has been read
            # if frames_remaining == 0:
            #     print(f"Requesting {frame_chunk_size} frames")
            #     frames_remaining = frame_chunk_size
            #     writable[0].send((frame_chunk_size).to_bytes(1, byteorder='big'))
            if perf_counter() - time_count > 1:
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
                # frames_remaining -= 1
                frames_recieved += 1
                # print(frames_recieved)

                # Interpret buffer as 1-D array with specified frame datatype
                buffer = data[:frame_info.size]
                a = np.frombuffer(buffer, dtype=frame_info.bit_depth)

                

                # Create Message depending on sensor image type
                if frame_info.format == 'RGB8':
                    # Swap color channels as OpenCV uses BGR not RGB
                    # make frame
                    image = a.reshape((frame_info.height, frame_info.width, frame_info.channels))
                    # msg
                    img_msg = bridge.cv2_to_imgmsg(image, "rgb8")

                if frame_info.format == 'Depth16':
                    # make frame
                    image = a.reshape((frame_info.height, frame_info.width, frame_info.channels))
                    # image = image / image.max() # dont know if necessary
                    img_msg = bridge.cv2_to_imgmsg(image, "mono16")

                if frame_info.format == 'Gray8':  # for infrared
                    # make frame
                    image = a.reshape((frame_info.height, frame_info.width, frame_info.channels))
                    # image = image / image.max() # dont know if necessary
                    img_msg = bridge.cv2_to_imgmsg(image, "mono8")

                if frame_info.format == 'XYZ':
                    depth_points_array = a.reshape((int(a.size / frame_info.channels), frame_info.channels))
                    
                    # make it 2d (even if height will be 1)
                    # convert to structured array
                    dty = np.dtype(
                        [('x', 'float32'), ('y', 'float32'), ('z', 'float32')])
                    depth_points = np.zeros(depth_points_array.shape[0], dtype=dty)
                    depth_points['x'] = depth_points_array[:, 0]
                    depth_points['y'] = depth_points_array[:, 1]
                    depth_points['z'] = depth_points_array[:, 2]
                    depth_points = np.atleast_2d(depth_points)

                    img_msg = PointCloud2()
                    img_msg.height = depth_points.shape[0]
                    img_msg.width = depth_points.shape[1]
                    img_msg.fields = arr_to_fields(depth_points)
                    img_msg.is_bigendian = False # assumption
                    img_msg.point_step = depth_points.dtype.itemsize
                    img_msg.row_step = img_msg.point_step*depth_points.shape[1]
                    img_msg.is_dense = all([np.isfinite(depth_points[fname]).all() for fname in depth_points.dtype.names])
                    img_msg.data = depth_points.tobytes()

                img_msg.header.stamp = image_pub.get_clock().now().to_msg()
                img_msg.header.frame_id = frameid

                image_pub.publisher_.publish(img_msg)

                # rate.sleep()
                if len(select.select([sock], [sock], [], 1)[0]):
                    # delete the rest of the buffer in the socket
                    data = bytearray(read_chunk_size)
                    size = readable[0].recv_into(data, read_chunk_size)
                    # and socket exists (use select) why do you require this
                    while size == read_chunk_size and len(select.select([sock], [sock], [], 1)[0]):
                        size = readable[0].recv_into(data, read_chunk_size)
                        pass
                    data = data[:size]

                # rospy.loginfo(cloud_msg)

# Main
def main(args=None):

    # Parse launch file arguments
    node_name = sys.argv[1]
    topic_name = sys.argv[2]
    frame_name = sys.argv[3]

    # initialize ROS2 node and publisher
    rclpy.init(args=args)
    image_publisher = ImagePublisher(node_name, topic_name)

    # Select desired topics
    # image_type = 'forward-tis-dfm27up/color/image-raw'
    if topic_name == 'tis_camera':
        image_type = 'forward-tis-dfm27up/color/image-raw'
    elif topic_name == 'forward_chest_rgb':
        image_type = 'forward-chest-realsense-d435/color/image-rect'
    elif topic_name == 'forward_chest_left_infrared':
        image_type = 'forward-chest-realsense-d435/left-infrared/image-rect'
    elif topic_name == 'forward_chest_right_infrared':
        image_type = 'forward-chest-realsense-d435/right-infrared/image-rect'
    elif topic_name == 'forward_chest_depth_image':
        image_type = 'forward-chest-realsense-d435/depth/image-rect'
    elif topic_name == 'forward_chest_depth_points':
        image_type = 'forward-chest-realsense-d435/depth/points'

    # publish
    bridge = CvBridge()
    port = asyncio.get_event_loop().run_until_complete(start_stream(image_type))
    process_stream_and_publish(image_type, port, image_publisher, bridge, frame_name)

if __name__ == '__main__':
    try:
        main()

    except websockets.exceptions.ConnectionClosedError:
        sys.exit("Connection to robot lost")
    except (ConnectionRefusedError, asyncio.exceptions.TimeoutError):
        sys.exit("Could not connect to robot")
    except KeyboardInterrupt:
        sys.exit("Caught user exit")
