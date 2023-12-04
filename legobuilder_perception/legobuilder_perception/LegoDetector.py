import os
import cv2
from sensor_msgs.msg import Image
from legobuilder_interfaces.msg import BoundingBox
from cv_bridge import CvBridge

from detectron2.data.datasets import register_coco_instances
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.engine import DefaultTrainer
from detectron2.config import get_cfg
from detectron2.utils.visualizer import ColorMode

from detectron2.structures import Instances
from typing import List, Tuple

register_coco_instances("lego", {}, "./datasets/lego/annotations.json", "./datasets/lego/")
lego_metadata = MetadataCatalog.get("lego")
dataset_dicts = DatasetCatalog.get("lego")


class LegoDetector:

    def __init__(self):

        self.cfg = get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.MODEL.WEIGHTS = os.path.join("./datasets/lego/model_final.pth")
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5   # set the testing threshold for this model
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1
        self.cfg.DATASETS.TEST = ("lego", )
        self.predictor = DefaultPredictor(self.cfg)
        self.bridge = CvBridge()

        self.confidence_threshold = 0.9

    def detect(self, img : Image) -> tuple[Image, list[BoundingBox], list[float]]:
        # Convert message to CV image
        cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        im = cv_image[1600:2624, 2400:3424]

        # Get detectron inference
        outputs = self.predictor(im)
        instances = outputs['instances'].to("cpu")
        confident_instances = instances[instances.scores > self.confidence_threshold]

        v = Visualizer(
            im[:,:,::-1],
            metadata=lego_metadata,
            scale=1,
            instance_mode=ColorMode.IMAGE_BW # remove the colors of unsegmented pixels    
        )
        v = v.draw_instance_predictions(confident_instances)
        
        # Set the annotated image
        ann_img = self.bridge.cv2_to_imgmsg(v.get_image())

        # Set boxes and scores msg
        boxes = []
        scores = []
        for i in range(len(confident_instances)):
            b = BoundingBox()
            b.x_min = confident_instances.pred_box[i, 0]
            b.y_min = confident_instances.pred_box[i, 1]
            b.x_max = confident_instances.pred_box[i, 2]
            b.y_max = confident_instances.pred_box[i, 3]
            boxes[i] = b
            scores[i] = confident_instances.scores[i]


        #cv2.imshow('original_image',im)
        #cv2.imshow('prediction',v.get_image()[:, :, ::-1])
        #cv2.waitKey(1)

        return ann_img, boxes, scores