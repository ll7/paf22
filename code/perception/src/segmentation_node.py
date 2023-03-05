#!/usr/bin/env python3

import pathlib
import numpy as np
import ros_compatibility as roscomp
import torch
from rospy.numpy_msg import numpy_msg
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import Image

from panoptic_segmentation.efficientps import EffificientPS as EfficientPS
from panoptic_segmentation.train_net import add_custom_param

from detectron2.config import get_cfg

CFG_FILE_PATH = pathlib.Path(
    __file__).parent / "panoptic_segmentation" / "config.yaml"
MODEL_PATH = pathlib.Path(
    __file__).parent.parent / \
             "models/panoptic_segmentation/efficientps-32.ckpt"


class SegmentationNode(CompatibleNode):
    """
    This node runs the panoptic segmentation model on
    the camera images and publishes the segmented results.
    """

    SIDES = ["Left", "Right", "Back", "Center"]

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        self.publishers = {}
        self.loginfo("Initializing panoptic segmentation node...")

        self.role_name = self.get_param("role_name", "hero")

        self.model = self.load_model()

        self.setup_camera_subscriptions()

    def setup_camera_subscriptions(self):
        for side in self.SIDES:
            self.new_subscription(
                msg_type=numpy_msg(Image),
                callback=lambda image: self.handle_camera_image(image, side),
                topic=f"/carla/{self.role_name}/{side}/image",
                qos_profile=10
            )

    def setup_camera_publishers(self):
        self.publishers = {}
        for side in self.SIDES:
            self.publishers[side] = self.new_publisher(
                msg_type=Image,
                topic=f"/paf/{self.role_name}/{side}/segmented_image",
                qos_profile=10
            )

    @staticmethod
    def load_model():
        cfg = get_cfg()
        add_custom_param(cfg)
        cfg.merge_from_file(str(CFG_FILE_PATH))

        return EfficientPS.load_from_checkpoint(
            cfg=cfg,
            checkpoint_path=str(MODEL_PATH)
        )

    def predict(self, image: np.ndarray):
        self.loginfo(f"predicting image shape: {image.shape}")
        # expand
        image = np.expand_dims(image, axis=0)
        prediction = self.model(torch.from_numpy(image)).to_numpy()
        self.loginfo(f"predictions: {prediction.shape}")
        return prediction

    def handle_camera_image(self, image, side):
        self.loginfo(f"got image from {side} camera")
        image_array = np.frombuffer(image.data, dtype=np.uint8)
        image_array = image_array.reshape((image.height, image.width, -1))
        # remove alpha channel
        image_array = image_array[:, :, :3]

        # image = image_array.reshape((image.height, image.width, 3))

        # self.loginfo(f"image shape: {image.data.shape}")

        prediction = self.predict(image_array)
        self.publishers[side].publish(prediction)
        self.loginfo(f"prediction shape: {prediction.shape}")

    def run(self):
        self.spin()
        pass
        # while True:
        #    self.spin()


if __name__ == "__main__":
    roscomp.init("SegmentationNode")
    # try:
    node = SegmentationNode("SegmentationNode")
    node.run()
    # except KeyboardInterrupt:
    #    pass
    # finally:
#       roscomp.shutdown()
#
