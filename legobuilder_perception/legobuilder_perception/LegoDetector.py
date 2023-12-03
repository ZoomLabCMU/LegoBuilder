import os
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from detectron2.data.datasets import register_coco_instances
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.engine import DefaultTrainer
from detectron2.config import get_cfg
from detectron2.utils.visualizer import ColorMode

register_coco_instances("lego", {}, "./datasets/lego/annotations.json", "./datasets/lego/")
lego_metadata = MetadataCatalog.get("lego")
dataset_dicts = DatasetCatalog.get("lego")


class Detectron2:

    def __init__(self):

        self.cfg = get_cfg()
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
        self.cfg.MODEL.WEIGHTS = os.path.join("./datasets/lego/model_final.pth")
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5   # set the testing threshold for this model
        self.cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1
        self.cfg.DATASETS.TEST = ("lego", )
        self.predictor = DefaultPredictor(self.cfg)
        self.bridge = CvBridge()

        self.top_bbox = [374, 24, 700, 270]
        self.bottom_bbox = [378, 378, 659, 672]
        self.score_threshold = 0.9

        self.starting_top = 0
        self.starting_bottom = 0
        self.ending_top = 0
        self.ending_bottom = 0

    def get_top_bbox(self):
        return self.top_bbox

    def get_bottom_bbox(self):
        return self.bottom_bbox

    def set_top_bbox(self):
        global clicks

        img_msg = rospy.wait_for_message('/rgb/image_raw',Image)

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        rgb_image = cv_image[1600:2624, 2400:3424]

        clicks = []

        def rectangle_click(event, x, y, flags, param):
            # if the left mouse button was clicked, record the starting
            # (x, y) coordinates and indicate that cropping is being
            # performed
            global clicks

            if event == cv2.EVENT_LBUTTONDOWN:
                clicks = [(x, y)]
                print(x,y)
            # check to see if the left mouse button was released
            elif event == cv2.EVENT_LBUTTONUP:
                # record the ending (x, y) coordinates and indicate that
                # the cropping operation is finished
                clicks.append((x, y))
                print(x,y)
                temp_rgb_image = rgb_image.copy()
                cv2.rectangle(temp_rgb_image, clicks[0], clicks[1], (0, 255, 0), 2)
                cv2.imshow("top_bbox", temp_rgb_image)
            
        cv2.namedWindow('top_bbox')
        cv2.imshow('top_bbox', rgb_image)
        cv2.setMouseCallback('top_bbox', rectangle_click)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        self.top_bbox = [min(clicks[0][0],clicks[1][0]), min(clicks[0][1],clicks[1][1]), 
                         max(clicks[0][0],clicks[1][0]), max(clicks[0][1],clicks[1][1])]
        print(self.top_bbox)

    def set_bottom_bbox(self):
        global clicks

        img_msg = rospy.wait_for_message('/rgb/image_raw',Image)

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        rgb_image = cv_image[1600:2624, 2400:3424]

        clicks = []

        def rectangle_click(event, x, y, flags, param):
            # if the left mouse button was clicked, record the starting
            # (x, y) coordinates and indicate that cropping is being
            # performed
            
            global clicks

            if event == cv2.EVENT_LBUTTONDOWN:
                clicks = [(x, y)]
                print(x,y)
            # check to see if the left mouse button was released
            elif event == cv2.EVENT_LBUTTONUP:
                # record the ending (x, y) coordinates and indicate that
                # the cropping operation is finished
                clicks.append((x, y))
                print(x,y)
                temp_rgb_image = rgb_image.copy()
                cv2.rectangle(temp_rgb_image, clicks[0], clicks[1], (0, 255, 0), 2)
                cv2.imshow("bottom_bbox", temp_rgb_image)
                
            
        cv2.namedWindow('bottom_bbox')
        cv2.imshow('bottom_bbox', rgb_image)
        cv2.setMouseCallback('bottom_bbox', rectangle_click)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        self.bottom_bbox = [min(clicks[0][0],clicks[1][0]), min(clicks[0][1],clicks[1][1]), 
                            max(clicks[0][0],clicks[1][0]), max(clicks[0][1],clicks[1][1])]
        print(self.bottom_bbox)

    def evaluate_start_image(self):

        img_msg = rospy.wait_for_message('/rgb/image_raw',Image)

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        im = cv_image[1600:2624, 2400:3424]

        outputs = self.predictor(im)

        outputs_on_cpu = outputs['instances'].to("cpu")
        bounding_boxes = outputs_on_cpu.pred_boxes
        scores = outputs_on_cpu.scores.numpy()

        ind = 0
        self.starting_top = 0
        self.starting_bottom = 0

        for i in bounding_boxes.__iter__():
            current_score = scores[ind]
            ind += 1
            if current_score >= self.score_threshold:
                bbox = i.numpy()
                if bbox[0] >= self.top_bbox[0] and bbox[1] >= self.top_bbox[1] and bbox[2] <= self.top_bbox[2] and bbox[3] <= self.top_bbox[3]:
                    self.starting_top += 1
                if bbox[0] >= self.bottom_bbox[0] and bbox[1] >= self.bottom_bbox[1] and bbox[2] <= self.bottom_bbox[2] and bbox[3] <= self.bottom_bbox[3]:
                    self.starting_bottom += 1

        print("Starting Top : " + str(self.starting_top))
        print("Starting Bottom : " + str(self.starting_bottom))

        #cv2.imshow('original_image',im)
        v = Visualizer(im[:, :, ::-1],
                       metadata=lego_metadata, 
                       scale=1, 
                       instance_mode=ColorMode.IMAGE_BW   # remove the colors of unsegmented pixels
        )
        v = v.draw_instance_predictions(outputs["instances"].to("cpu"))
        cv2.imshow('prediction',v.get_image()[:, :, ::-1])
        cv2.waitKey(1)

        return (self.starting_top, self.starting_bottom)

    def evaluate_end_image(self):
        img_msg = rospy.wait_for_message('/rgb/image_raw',Image)

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        im = cv_image[1600:2624, 2400:3424]

        outputs = self.predictor(im)

        outputs_on_cpu = outputs['instances'].to("cpu")
        bounding_boxes = outputs_on_cpu.pred_boxes
        scores = outputs_on_cpu.scores.numpy()

        ind = 0
        self.ending_top = 0
        self.ending_bottom = 0

        for i in bounding_boxes.__iter__():
            current_score = scores[ind]
            ind += 1
            if current_score >= self.score_threshold:
                bbox = i.numpy()
                if bbox[0] >= self.top_bbox[0] and bbox[1] >= self.top_bbox[1] and bbox[2] <= self.top_bbox[2] and bbox[3] <= self.top_bbox[3]:
                    self.ending_top += 1
                if bbox[0] >= self.bottom_bbox[0] and bbox[1] >= self.bottom_bbox[1] and bbox[2] <= self.bottom_bbox[2] and bbox[3] <= self.bottom_bbox[3]:
                    self.ending_bottom += 1

        print("Ending Top : " + str(self.ending_top))
        print("Ending Bottom : " + str(self.ending_bottom))

        if self.starting_top == self.ending_top and self.starting_bottom == self.ending_bottom:
            print('Failed to pick or place block.')
        elif self.starting_top < self.ending_top and self.starting_bottom > self.ending_bottom:
            print('Successfully picked up ' + str(self.ending_top - self.starting_top) + ' block(s).')
        elif self.starting_top > self.ending_top and self.starting_bottom < self.ending_bottom:
            print('Successfully placed ' + str(self.ending_bottom - self.starting_bottom) + ' block(s).')
        else:
            print('Some error has occurred.')

        #cv2.imshow('original_image',im)
        v = Visualizer(im[:, :, ::-1],
                       metadata=lego_metadata, 
                       scale=1, 
                       instance_mode=ColorMode.IMAGE_BW   # remove the colors of unsegmented pixels
        )
        v = v.draw_instance_predictions(outputs["instances"].to("cpu"))
        cv2.imshow('prediction',v.get_image()[:, :, ::-1])
        cv2.waitKey(1)

        return (self.starting_top, self.starting_bottom, self.ending_top, self.ending_bottom)