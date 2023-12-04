import rclpy
from rclpy.node import Node
from legobuilder_interfaces.srv import LegoDetection

from legobuilder_perception.LegoDetector import LegoDetector


class LegoDetectorNode(Node):
    def __init__(self):
        super().__init__('lego_detector_node')
        
        self.lego_detector = LegoDetector()

        self.lego_detection_srv = self.create_service(
            LegoDetection,
            "/lego_detection",
            self.handle_lego_detection_req,
            1
        )

    def handle_lego_detection_req(self, request : LegoDetection.Request, response : LegoDetection.Response) -> LegoDetection.Response :
        img  = request.img
        self.get_logger().info(
            f'Image recieved for inference using Detectron2'
        )
        ann_img, bboxes, scores = self.lego_detector.detect(img)
        response.ann_img = ann_img
        response.bounding_boxes = bboxes
        response.scores = scores
        self.get_logger().info(
            f'Lego detection instances returned'
        )
        return response
        
def main(args=None):
    rclpy.init(args=args)

    lego_detector_node = LegoDetectorNode()

    rclpy.spin(lego_detector_node)

    lego_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

