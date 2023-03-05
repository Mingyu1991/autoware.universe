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
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from autoware_auto_perception_msgs.msg import TrafficLightRoiArray, TrafficLightRoi, TrafficSignal, TrafficSignalArray, TrafficLight
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
from cv_bridge import CvBridge
from mmcls.apis import init_model, inference_model
import mmcv
import os
import shutil
import numpy as np
import copy
import time
from typing import List


MODEL = dict(
            ckpt="/home/mingyuli/workspace/pilot-auto.xx1/src/autoware/universe/perception/traffic_light_classifier_py/traffic_light_classifier_py/models/efficientnet-b0/efficientnet-b0.pth",
            cfg="/home/mingyuli/workspace/pilot-auto.xx1/src/autoware/universe/perception/traffic_light_classifier_py/traffic_light_classifier_py/models/efficientnet-b0/efficientnet-b0.py"
    )

LABEL_TO_COLOR = dict(
    red = TrafficLight.RED,
    yellow = TrafficLight.AMBER,
    green = TrafficLight.GREEN,
    white = TrafficLight.WHITE,
)

LABEL_TO_SHAPE = dict(
    left = TrafficLight.LEFT_ARROW,
    right = TrafficLight.RIGHT_ARROW,
    straight = TrafficLight.UP_ARROW,
    leftdiagonal = TrafficLight.DOWN_LEFT_ARROW,
    rightdiagonal = TrafficLight.DOWN_RIGHT_ARROW,
    
    unknown = TrafficLight.UNKNOWN,
)

class TrafficLightClassifier(Node):

    def __init__(self):
        super().__init__('traffic_light_classifier_py')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.SYSTEM_DEFAULT,
            depth=1,
        )
        image_topic = "/sensing/camera/traffic_light/image_raw"
        roi_topic = "/perception/traffic_light_recognition/rois"
        image_sub = message_filters.Subscriber(self, Image, image_topic, qos_profile=qos_profile)
        roi_sub = message_filters.Subscriber(self, TrafficLightRoiArray, roi_topic)
        ts = message_filters.TimeSynchronizer([image_sub, roi_sub], 100)
        ts.registerCallback(self.callback)
        self.publisher_ = self.create_publisher(TrafficSignalArray, "/perception/traffic_light_recognition/classified/traffic_signals", 10)
        self.bridge = CvBridge()
        self.model = init_model(MODEL["cfg"], MODEL["ckpt"])

    def callback(self, image_msg: Image, roi_msg: TrafficLightRoiArray):
        output = TrafficSignalArray()
        output.header = roi_msg.header
        image = self.bridge.imgmsg_to_cv2(image_msg)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        t1 = time.time()
        debug_img = copy.deepcopy(image)
        for roi in roi_msg.rois:
            signal = TrafficSignal()
            signal.map_primitive_id = roi.id
            roi: TrafficLightRoi
            x1 = roi.roi.x_offset
            y1 = roi.roi.y_offset
            x2 = roi.roi.x_offset + roi.roi.width
            y2 = roi.roi.y_offset + roi.roi.height
            if roi.roi.width <= 1 or roi.roi.height <= 1:
                continue
            image_roi = image[y1:y2, x1:x2, :]
            result = inference_model(self.model, image_roi)
            result_class = result["pred_class"]
            confidence = result["pred_score"]
            for label in result_class.split("-"):
                light = TrafficLight()
                if label == "unknown":
                    light.color = TrafficLight.UNKNOWN
                    light.shape = TrafficLight.UNKNOWN
                elif label in LABEL_TO_COLOR:
                    light.color = LABEL_TO_COLOR[label]
                    light.shape = TrafficLight.CIRCLE
                else:
                    light.color = TrafficLight.GREEN
                    light.shape = LABEL_TO_SHAPE[label]
                light.confidence = confidence
                signal.lights.append(light)
            output.signals.append(signal)
            
            debug_img = cv2.rectangle(debug_img, (x1, y1), (x2, y2), (255, 0, 0))
        self.publisher_.publish(output)
        # save_dir = "/tmp/images"
        # os.makedirs(save_dir, exist_ok=True)
        # save_path = os.path.join(save_dir, f"{roi_msg.header.stamp.sec}_{roi_msg.header.stamp.nanosec}.jpg")
        # debug_img = cv2.resize(debug_img, (debug_img.shape[1] // 2, debug_img.shape[0] // 2))
        # cv2.imwrite(save_path, debug_img)
        # print(f"inference time = {time.time() - t1}")
                    

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightClassifier()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
