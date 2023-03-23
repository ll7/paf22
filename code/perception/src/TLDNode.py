#!/usr/bin/env python3

import pathlib
import numpy as np
import ros_compatibility as roscomp
from rospy.numpy_msg import numpy_msg
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from traffic_light_detection.src.traffic_light_detection.\
    traffic_light_inference import TrafficLightInference
from panoptic_segmentation.preparation.labels import name2label
from panoptic_segmentation.datasets.panoptic_dataset import rgb2id
from panopticapi.utils import id2rgb

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

    def handle_segmented_image(self, image):
        self.loginfo(f"TLDNode got segmented image from EfficientPS "
                     f"{self.side}")
        image_array = np.frombuffer(image.data, dtype=np.uint8)
        image_array = image_array.reshape((image.height, image.width, -1))
        image = rgb2id(image_array)
        mask = np.ma.masked_inside(image, self.traffic_light_id * 1000,
                                   (self.traffic_light_id + 1) * 1000 - 1)
        mask = mask.mask

        tl_image = np.zeros(image.shape)
        tl_image[mask] = image[mask]
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

        if not mask.any():
            return

        indices = np.nonzero(tl_image[0:tl_image.shape[0] // 2,
                             tl_image.shape[1] // 3:
                             2 * (tl_image.shape[1] // 3)])
        upper_left = [min(indices[0]), min(indices[1])]
        lower_right = [max(indices[0]), max(indices[1])]

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
