import copy

import rospy
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
import cv2 as cv
from cv_bridge import CvBridge
import image_geometry
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import ros_numpy

class LidarUtility:

    latest_lidar_data = None
    latest_camera_info = None
    latest_camera_image = None
    one_shown = False

    def __init__(self):
        rospy.Subscriber("/carla/hero/LIDAR", PointCloud2, self.process_lidar_image)
        rospy.Subscriber("/carla/hero/Center/camera_info", CameraInfo, self.process_camera_info)
        rospy.Subscriber("/carla/hero/Center/image", Image, self.process_image)


    def process_lidar_image(self, data):
        rospy.loginfo('new lidar data received')
        self.latest_lidar_data = ros_numpy.numpify(data)

    def process_camera_info(self, data):
        rospy.loginfo('new camera info received')
        self.latest_camera_info = data

    def process_image(self, data):
        rospy.loginfo('new image received')
        self.latest_camera_image = data

    def getDepthAtPoint(self, x: int, y: int):
        print('')

    def processLidarImage(self, lidar_image):
        if self.one_shown is True:
            return

        if type(None) not in (type(self.latest_lidar_data), type(self.process_camera_info), type(self.latest_camera_image)):
            rospy.loginfo(self.extract_rgb(self.latest_lidar_data, self.latest_camera_image, self.latest_camera_info))

    # def getLidarImage(self) -> PointCloud2:
    #     lidar_data = rospy.wait_for_message("/carla/hero/LIDAR", PointCloud2)
    #     return lidar_data

    def extract_rgb(self, lidar_data, image, camera_info):
        bridge = CvBridge()
        camera_object = image_geometry.PinholeCameraModel()

        camera_object.fromCameraInfo(msg=camera_info)

        rvec = np.array([0., 0., 0.])
        tvec = np.array([0., 0., 0.])

        points = np.array(self.latest_lidar_data.view(np.float32).reshape(self.latest_lidar_data.shape + (-1,)))[:, :3]
        points = points.astype('float64')

        img_points, _ = cv.projectPoints(
            objectPoints=points,
            rvec=rvec,
            tvec=tvec,
            cameraMatrix=camera_object.intrinsicMatrix(),
            distCoeffs=camera_object.distortionCoeffs(),
        )

        print('-----------')
        print(img_points)
        img = np.zeros((camera_info.height, camera_info.width, 3), np.uint8)
        img[img_points[:, 1], img_points[:, 0]] = (0, 0, 255)
        self.one_shown = True

        cv.imshow("projected points", img)
        cv.waitKey(0)
