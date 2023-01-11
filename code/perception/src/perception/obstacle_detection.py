#!/usr/bin/env python
import tensorflow as tf
import numpy as np
import ros_compatibility as roscomp
from derived_object_msgs.msg import ObjectArray
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import Image


# Partially inspired by:
# https://github.com/erdos-project/pylot/blob/master/pylot/perception/detection/detection_operator.py
# ROS Node for Obstacle Detection
class ObstacleDetection(CompatibleNode):
    """
    This class is a ROS node for obstacles.
    It uses the camera data from the center camera,
    and tries to detect obstacles using a faster-rcnn model.
    """

    def __init__(self):
        super().__init__('obstacle_detection')
        self.loginfo('Obstacle Detection Node started')

        camera_topic = self.get_param('camera_topic',
                                      '/carla/hero/Center/image')

        self._camera_sub = self.new_subscription(Image,
                                                 camera_topic,
                                                 self._camera_callback, 10)
        self.obstacle_pub = self.new_publisher(ObjectArray,
                                               '/perception/obstacles', 10)

        self._model = tf.saved_model.load(
            '../../models/faster-rcnn')

    def _camera_callback(self, msg: Image):
        self.get_logger().info('I heard: "%s"' % msg.header.frame_id)
        result = self.__run_model(np.array(msg.data))
        print(result)

    def __run_model(self, image_np):
        # Expand dimensions since the model expects images to have
        # shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)

        infer = self._model.signatures['serving_default']
        result = infer(tf.convert_to_tensor(value=image_np_expanded))

        boxes = result['boxes']
        scores = result['scores']
        classes = result['classes']
        num_detections = result['detections']

        num_detections = int(num_detections[0])
        res_classes = [int(cls) for cls in classes[0][:num_detections]]
        res_boxes = boxes[0][:num_detections]
        res_scores = scores[0][:num_detections]
        return num_detections, res_boxes, res_scores, res_classes


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init('obstacle_detection', args=args)

    try:
        node = ObstacleDetection()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
