from v4l2py import Device

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import signal

class GracefulExiter():

    def __init__(self):
        self.state = False
        signal.signal(signal.SIGINT, self.change_state)

    def change_state(self, signum, frame):
        print("Exit flag set to True (Repeat to exit now)")
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        self.state = True

    def exit(self):
        return self.state

class WorkspaceCameraNode(Node):
    def __init__(self):
        super().__init__("workspace_camera_node") #type: ignore

        # Image Publisher
        self.img_publisher = self.create_publisher(
            Image,
            'workspace_img',
            10
        )
        self.bridge = CvBridge()

    def publish_frame(self, frame):
        jpg_as_np = np.frombuffer(bytes(frame), dtype=np.uint8)
        img = cv2.imdecode(jpg_as_np, flags=1)
        cv_image = self.bridge.cv2_to_imgmsg(img, 'bgr8')
        self.img_publisher.publish(cv_image)


def main(args=None):
    rclpy.init(args=args)

    workspace_camera_node = WorkspaceCameraNode()

    flag = GracefulExiter()

    try:
        with Device.from_id(0) as cam:
            for i, frame in enumerate(cam):
                workspace_camera_node.publish_frame(frame)
                if flag.exit():
                    break
    except FileNotFoundError as e:
        workspace_camera_node.get_logger().info(
            f'Workspace camera not found, shutting down'
        )

    workspace_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()