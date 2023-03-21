#!/usr/bin/env python3

import pathlib
import numpy as np
import ros_compatibility as roscomp
from rospy.numpy_msg import numpy_msg
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import Image

from traffic_light_detection.traffic_light_inference import \
    TrafficLightInference
from panoptic_segmentation.preparation.labels import name2label
from panoptic_segmentation.datasets.panoptic_dataset import rgb2id

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

        self.effps_sub = None
        self.camera_sub = None
        self.image = None
        self.publisher = None
        self.loginfo("Initializing traffic light detection node...")

        self.role_name = self.get_param("role_name", "hero")
        self.side = self.get_param("side", "Center")

        self.model = self.load_model()

        self.setup_subscriptions()
        self.setup_publishers()
        self.traffic_light_id = name2label("traffic light").id

    def setup_subscriptions(self):
        self.effps_sub = self.new_subscription(
            msg_type=numpy_msg(Image),
            callback=self.handle_segmented_image,
            topic=f"/carla/{self.role_name}/{self.side}/segmented_image",
            qos_profile=1
        )
        self.camera_sub = self.new_subscription(
            msg_type=numpy_msg(Image),
            callback=self.handle_image,
            topic=f"/carla/{self.role_name}/{self.side}/image",
            qos_profile=1
        )

    def setup_publishers(self):
        self.publisher = self.new_publisher(
            msg_type=str,
            topic=f"/paf/{self.role_name}/{self.side}/segmented_image",
            qos_profile=1
        )

    @staticmethod
    def load_model():
        model = TrafficLightInference(model_path=MODEL_PATH)
        return model

    def predict(self, image: np.ndarray):
        self.loginfo(f"predicting image shape: {image.shape}")
        result = self.model(image)
        result = result.cpu().numpy()
        return result

    def handle_image(self, image):
        image_array = np.frombuffer(image.data, dtype=np.uint8)
        image_array = image_array.reshape((image.height, image.width, -1))
        # remove alpha channel
        image_array = image_array[:, :, :3]
        self.image = image_array

    def handle_segmented_image(self, image):
        self.loginfo(f"got segmented image from EfficientPS {self.side}")
        image = rgb2id(image)
        mask = self.traffic_light_id * 1000 <= image < \
            (self.traffic_light_id + 1) * 1000
        if not mask.any():
            return

        tl_image = np.zeros(image.shape)
        tl_image[mask] = image[mask]

        indices = np.nonzero(tl_image[0:tl_image.shape[0] // 2,
                             tl_image.shape[1] // 3:
                             2 * (tl_image.shape[1] // 3)])
        upper_left = [min(indices[:, 0]), min(indices[: 1])]
        lower_right = [max(indices[:, 0]), max(indices[: 1])]

        traffic_light = self.image[upper_left[0]:lower_right[0],
                                   upper_left[1]:lower_right[1]]
        classification = self.predict(traffic_light)

        # construct the message
        msg = Image()
        msg.header.stamp = roscomp.ros_timestamp(
            self.get_time(), from_sec=True)
        msg.header.frame_id = "map"
        msg.data = str(classification)
        self.publisher.publish(msg)

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
