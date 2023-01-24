import rospy
from sensor_msgs.point_cloud2 import PointCloud2
import cv2 as cv


class LidarUtility:

    def getDepthAtPoint(self, x: int, y: int):
        print('')

    def processLidarImage(self, lidar_image):
        # https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga1019495a2c8d1743ed5cc23fa0daff8c

        # http://wiki.ros.org/image_geometry (maybe use this?)
        cv.projectPoints(
            lidar_image.data,
            lidar_image.R,
            None,
            lidar_image.K,
            lidar_image.D
        )

    def getLidarImage(self) -> PointCloud2:
        lidar_data = rospy.wait_for_message("/carla/hero/LIDAR", PointCloud2)
        return lidar_data
