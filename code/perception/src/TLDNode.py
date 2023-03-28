#!/usr/bin/env python3

import pathlib

import cv2
import numpy as np
import ros_compatibility as roscomp
from panopticapi.utils import id2rgb
from ros_compatibility.node import CompatibleNode
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
from std_msgs.msg import String

from panoptic_segmentation.datasets.panoptic_dataset import rgb2id
from panoptic_segmentation.preparation.labels import name2label
from traffic_light_detection.src.traffic_light_detection. \
    traffic_light_inference import TrafficLightInference

MODEL_PATH = pathlib.Path(
    __file__).parent.parent / \
             "models/traffic_light_detection/tld_ckpt.pt"


class TLDNode(CompatibleNode):
    """
    This node runs the traffic light detection model on
    the semantic images and publishes the classified traffic lights.
    """

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.instance_sub = None
        self.snip_publisher = None
        self.class_publisher = None
        self.loginfo("Initializing traffic light detection node...")
        self.effps_sub = None
        self.camera_sub = None
        self.image = None

        self.role_name = self.get_param("role_name", "hero")
        self.side = self.get_param("side", "Center")

        self.model = self.load_model()

        self.setup_subscriptions()
        self.setup_publishers()
        self.traffic_light_id = (name2label["traffic light"]).id
        self.loginfo("Traffic light detection node initialized.")

    def setup_subscriptions(self):
        self.effps_sub = self.new_subscription(
            msg_type=numpy_msg(Image),
            callback=self.handle_segmented_image,
            topic=f"/paf/{self.role_name}/{self.side}/segmented_image",
            qos_profile=1
        )
        self.camera_sub = self.new_subscription(
            msg_type=numpy_msg(Image),
            callback=self.handle_image,
            topic=f"/carla/{self.role_name}/{self.side}/image",
            qos_profile=1
        )
        """Include this code to compute the output of a segmentation camera"""
        # self.instance_sub = self.new_subscription(
        #     msg_type=numpy_msg(Image),
        #     callback=self.handle_instance_image,
        #     topic=f"/carla/{self.role_name}/Instance_{self.side}/image",
        #     qos_profile=1
        # )

    def setup_publishers(self):
        self.class_publisher = self.new_publisher(
            msg_type=String,
            topic=f"/paf/{self.role_name}/{self.side}/traffic_light",
            qos_profile=1
        )
        self.snip_publisher = self.new_publisher(
            msg_type=numpy_msg(Image),
            topic=f"/paf/{self.role_name}/{self.side}/snipped_traffic_light",
            qos_profile=1
        )

    @staticmethod
    def load_model():
        model = TrafficLightInference(model_path=MODEL_PATH)
        return model

    def predict(self, image: np.ndarray):
        self.loginfo(f"TLDNode predicting image shape: {image.shape}")
        result = self.model(image)
        return result

    def handle_image(self, image):
        self.loginfo(f"TLDNode got image from camera {self.side}")
        image_array = np.frombuffer(image.data, dtype=np.uint8)
        image_array = image_array.reshape((image.height, image.width, -1))
        # remove alpha channel
        image_array = image_array[:, :, :3]
        image_array = cv2.resize(image_array, (1280, 720),
                                 interpolation=cv2.INTER_NEAREST)
        self.image = image_array

    def handle_instance_image(self, image):
        pass
        """Include this code to compute the output of a segmentation camera"""
        # self.loginfo(f"TLDNode got segmented image from Camera"
        #              f"{self.side}")
        # image_array = np.frombuffer(image.data, dtype=np.uint8)
        # image_array = image_array.reshape((image.height, image.width, -1))
        # image_array = image_array[:, :, :3]
        #
        # panoptic = np.zeros(image_array.shape, dtype=np.uint8)
        # formatted = image_array.reshape(-1, image_array.shape[2])
        # segmentIds = np.unique(formatted, axis=0)
        # instance_ids = np.zeros((max(segmentIds[:, 0]) + 1), dtype=np.uint8)
        # for segmentId in segmentIds:
        #     semanticId = segmentId[0]
        #     labelInfo = id2label[semanticId]
        #     if labelInfo.hasInstances:
        #         instance_id = 1000 * segmentId[0] \
        #                       + instance_ids[segmentId[0]]
        #         instance_ids[segmentId[0]] += 1
        #     else:
        #         instance_id = segmentId[0]
        #
        #     if labelInfo.ignoreInEval:
        #         continue
        #     mask = image_array == segmentId
        #     mask = mask.all(axis=2)
        #     color = [instance_id % 256, instance_id // 256,
        #              instance_id // 256 // 256]
        #     panoptic[mask] = color
        # image = rgb2id(panoptic)
        #
        # tld_id = self.traffic_light_id
        # tl_image = np.ma.masked_inside(image, tld_id * 1000,
        #                                (tld_id + 1) * 1000 - 1) \
        #     .filled(0)
        #
        # msg = Image()
        # msg.header.stamp = roscomp.ros_timestamp(
        #     self.get_time(), from_sec=True)
        # msg.header.frame_id = "map"
        # msg.height = 720
        # msg.width = 1280
        # msg.encoding = "rgb8"
        # msg.is_bigendian = 0
        # msg.step = 1280 * 3
        # msg.data = id2rgb(tl_image).tobytes()
        # self.snip_publisher.publish(msg)
        #
        # areas = {}
        # for instance in np.unique(tl_image):
        #     inst = np.ma.masked_not_equal(tl_image, instance).filled(0)
        #     indices = np.nonzero(inst[0:inst.shape[0] // 2,
        #                          inst.shape[1] // 4:
        #                          3 * (inst.shape[1] // 4)])
        #     upper_left = [min(indices[0]), min(indices[1])]
        #     lower_right = [max(indices[0]), max(indices[1])]
        #     areas[str(instance)] = [(lower_right[0] - upper_left[0]) *
        #                             (lower_right[1] - upper_left[1]),
        #                             upper_left,
        #                             lower_right]
        # if len(areas) > 0:
        #     maximum = max(areas, key=areas.get)
        #     upper_left = areas[maximum][1]
        #     lower_right = areas[maximum][2]
        #     traffic_light = self.image[upper_left[0]:lower_right[0],
        #                                upper_left[1]:lower_right[1]]
        #     classification = self.predict(traffic_light)
        #
        #     # construct the message
        #     self.class_publisher.publish(str(classification))
        #     self.loginfo(f"TLDNode classified traffic light "
        #                  f"{self.side}")

    def handle_segmented_image(self, image):
        self.loginfo(f"TLDNode got segmented image from EfficientPS "
                     f"{self.side}")
        image_array = np.frombuffer(image.data, dtype=np.uint8)
        image_array = image_array.reshape((image.height, image.width, -1))
        image = rgb2id(image_array)
        tl_image = np.ma.masked_inside(image, self.traffic_light_id * 1000,
                                       (self.traffic_light_id + 1) * 1000 - 1)\
            .filled(0)

        msg = Image()
        msg.header.stamp = roscomp.ros_timestamp(
            self.get_time(), from_sec=True)
        msg.header.frame_id = "map"
        msg.height = 720
        msg.width = 1280
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = 1280 * 3
        msg.data = id2rgb(tl_image).tobytes()
        self.snip_publisher.publish(msg)

        areas = {}
        for instance in np.unique(tl_image):
            inst = np.ma.masked_not_equal(tl_image, instance).filled(0)
            indices = np.nonzero(inst[0:inst.shape[0] // 2,
                                 inst.shape[1] // 4:
                                 3 * (inst.shape[1] // 4)])
            upper_left = [min(indices[0]), min(indices[1])]
            lower_right = [max(indices[0]), max(indices[1])]
            areas[str(instance)] = [(lower_right[0] - upper_left[0]) *
                                    (lower_right[1] - upper_left[1]),
                                    upper_left,
                                    lower_right]
        if len(areas) > 0:
            maximum = max(areas, key=areas.get)
            upper_left = areas[maximum][1]
            lower_right = areas[maximum][2]
            traffic_light = self.image[upper_left[0]:lower_right[0],
                                       upper_left[1]:lower_right[1]]
            classification = self.predict(traffic_light)

            # construct the message
            self.class_publisher.publish(str(classification))
            self.loginfo(f"TLDNode classified traffic light "
                         f"{self.side}")

    def run(self):
        self.spin()
        pass
        # while True:
        #    self.spin()


if __name__ == "__main__":
    roscomp.init("TLDNode")
    # try:

    node = TLDNode("TLDNode")
    node.run()
    # except KeyboardInterrupt:
    #    pass
    # finally:
#       roscomp.shutdown()
#
