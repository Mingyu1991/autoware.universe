# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import String
from autoware_auto_perception_msgs.msg import TrafficLightRoiArray, TrafficLightRoi
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
from cv_bridge import CvBridge
from mmdet.apis import inference_detector, init_detector
import mmcv
import os
import shutil
import numpy as np
import copy
import time
from typing import List


MODELS = [
    dict(
        ckpt="/home/mingyuli/workspace/pilot-auto.xx1/src/autoware/universe/perception/traffic_light_fine_detector_py/traffic_light_fine_detector_py/models/yolox-s/yolox-s.pth",
        cfg="/home/mingyuli/workspace/pilot-auto.xx1/src/autoware/universe/perception/traffic_light_fine_detector_py/traffic_light_fine_detector_py/models/yolox-s/yolox_s_8x8_300e_voc_tlr.py"
    ),
    dict(
        ckpt="/home/mingyuli/workspace/pilot-auto.xx1/src/autoware/universe/perception/traffic_light_fine_detector_py/traffic_light_fine_detector_py/models/yolox-l/yolox-l.pth",
        cfg="/home/mingyuli/workspace/pilot-auto.xx1/src/autoware/universe/perception/traffic_light_fine_detector_py/traffic_light_fine_detector_py/models/yolox-l/yolox_l_8x8_300e_voc_tlr.py"
    ),
    dict(
        ckpt="/home/mingyuli/workspace/pilot-auto.xx1/src/autoware/universe/perception/traffic_light_fine_detector_py/traffic_light_fine_detector_py/models/ssd-vgg-512/ssd-vgg-512.pth",
        cfg="/home/mingyuli/workspace/pilot-auto.xx1/src/autoware/universe/perception/traffic_light_fine_detector_py/traffic_light_fine_detector_py/models/ssd-vgg-512/ssd_vgg16_512x512_tlr_voc.py"
    ),
    dict(
        ckpt="/home/mingyuli/workspace/pilot-auto.xx1/src/autoware/universe/perception/traffic_light_fine_detector_py/traffic_light_fine_detector_py/models/ssd-vgg-300/ssd-vgg-300.pth",
        cfg="/home/mingyuli/workspace/pilot-auto.xx1/src/autoware/universe/perception/traffic_light_fine_detector_py/traffic_light_fine_detector_py/models/ssd-vgg-300/ssd_vgg16_300x300_tlr_voc.py"
    ),
]

SCORE_THRES = 0.3

class TrafficLightFineDetector(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.SYSTEM_DEFAULT,
            depth=1,
        )
        image_topic = "/sensing/camera/traffic_light/image_raw"
        rough_roi_topic = "/perception/traffic_light_recognition/rough/rois"
        image_sub = message_filters.Subscriber(self, Image, image_topic, qos_profile=qos_profile)
        rough_roi_sub = message_filters.Subscriber(self, TrafficLightRoiArray, rough_roi_topic)
        ts = message_filters.TimeSynchronizer([image_sub, rough_roi_sub], 10)
        ts.registerCallback(self.callback)
        self.publisher_ = self.create_publisher(TrafficLightRoiArray, "/perception/traffic_light_recognition/rois", 10)
        self.bridge = CvBridge()
        self.models = [
            init_detector(model["cfg"], model["ckpt"]) for model in MODELS
        ]

    def merge_detections(self, rough_roi: TrafficLightRoi, detections: List[TrafficLightRoi], image: np.ndarray)->TrafficLightRoi:
        det = TrafficLightRoi()
        cx = np.mean([det.roi.x_offset + det.roi.width / 2 for det in detections])
        cy = np.mean([det.roi.y_offset + det.roi.height / 2 for det in detections])
        x1 = cx - rough_roi.expect_width / 2
        x2 = cx + rough_roi.expect_width / 2
        y1 = cy - rough_roi.expect_height / 2
        y2 = cy + rough_roi.expect_height / 2
        x1 = int(np.clip(x1, 0, image.shape[1] - 1))
        x2 = int(np.clip(x2, 0, image.shape[1] - 1))
        y1 = int(np.clip(y1, 0, image.shape[0] - 1))
        y2 = int(np.clip(y2, 0, image.shape[0] - 1))
        det.roi.x_offset = x1
        det.roi.y_offset = y1
        det.roi.width = x2 - x1
        det.roi.height = y2 - y1
        det.id = rough_roi.id
        return det
        

    def callback(self, image_msg: Image, rough_roi_msg: TrafficLightRoiArray):
        output = copy.deepcopy(rough_roi_msg)
        output.rois.clear()
        image = self.bridge.imgmsg_to_cv2(image_msg)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        for roi in rough_roi_msg.rois:
            t1 = time.time()
            roi: TrafficLightRoi
            x1 = roi.roi.x_offset
            y1 = roi.roi.y_offset
            x2 = roi.roi.x_offset + roi.roi.width
            y2 = roi.roi.y_offset + roi.roi.height
            image_roi = image[y1:y2, x1:x2, :]
            detections = []
            for model in self.models:
                result = inference_detector(model, image_roi)
                bboxes, scores = result[1][:, :4], result[1][:, 4]
                if scores.shape[0] > 0:
                    max_score_index = np.where(scores == max(scores))[0][0]
                    if scores[max_score_index] >= SCORE_THRES:
                        best_bbox = bboxes[max_score_index]
                        output_roi = TrafficLightRoi()
                        output_roi.id = roi.id
                        output_roi.roi.x_offset = int(best_bbox[0] + roi.roi.x_offset)
                        output_roi.roi.y_offset = int(best_bbox[1] + roi.roi.y_offset)
                        output_roi.roi.width = int(best_bbox[2] - best_bbox[0])
                        output_roi.roi.height = int(best_bbox[3] - best_bbox[1])
                        detections.append(output_roi)
            if detections.__len__() > 0:
                output.rois.append(self.merge_detections(roi, detections, image))    
            
        self.publisher_.publish(output)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightFineDetector()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
