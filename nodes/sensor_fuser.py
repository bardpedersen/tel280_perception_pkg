#!/usr/bin/env python3
import rospy
import numpy as np
from tel280_perception_pkg.helper import SimpleNavHelpers
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from message_filters import Subscriber, ApproximateTimeSynchronizer
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math


class SensorFuserNode():

    def __init__(self):

        # Use ApproximateTimeSynchronizer to get synced sensor data
        atss = ApproximateTimeSynchronizer([Subscriber("/camera/rgb/image_raw", Image),
                                           Subscriber(
                                               "/camera/rgb/camera_info", CameraInfo),
                                           Subscriber("/scan", LaserScan)],
                                           queue_size=1, slop=0.1
                                           )
        atss.registerCallback(self.listener_callback)

        # 4 publishing laser scan projected image
        self.image_pub = rospy.Publisher("new", Image)

        # 4 converting sensor_msgs::Image -> opencv image
        self.bridge = CvBridge()

        # 4 Transforming laser points from LIDAR frame to camera frame
        self.helper = SimpleNavHelpers()

    def listener_callback(self, image: Image, camerainfo: CameraInfo, laser: LaserScan):

        rospy.loginfo(str("Recieved synced Image, CameraInfo, and LaserScan"))

        try:
            # convert sensor_msgs::Image -> opencv image
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            # Let us know if it was a failure
            print(e)

        # Read and store Laserscan
        # convert to x, y, z
        curr_angle = 0.0
        points = []
        ranges = []
        for range in laser.ranges:
            curr_point = [.0, .0, .0]  # x, y, z
            curr_point[0] = range * math.cos(curr_angle)
            curr_point[1] = range * math.sin(curr_angle)
            curr_point[2] = 0  # z is zero since this is a 2D LIDAR

            # FOV of camera is limited in between -45 , +45 degrees
            # we wont project points that are behind camera
            if curr_point[0] < 0 or math.isnan(range):
                curr_angle += laser.angle_increment
                continue

            # Narrow down on the sides as well, camera wont see too far left and right
            if abs(curr_point[1]) > 2:
                curr_angle += laser.angle_increment
                continue
            # store the points and keep a copy of ranges
            points.append(curr_point)
            ranges.append(range)

            # increment the angle to convert next laser range to x, y
            curr_angle += laser.angle_increment

        # points are now in laser frame, but we wanna project them to image, the first thing
        # we do is to transfrom them to 3D camera frame (camera_rgb_optical_frame) and then to image plane
        # Below function will transform points from source(lidar) to target(camera) frames
        points_in_cam_frame = self.helper.transform_points(
            points=points,
            source_frame=laser.header.frame_id,
            target_frame=image.header.frame_id)

        # Now we have points in 3D camera frame, lets project them to Image plane with following

        # x = P * X, where;
        #   x = [u v w]' 2D image point,
        #   P = Projection/camera matrix
        #       [fx'  0  cx' Tx]
        #   P = [ 0  fy' cy' Ty]
        #       [ 0   0   1   0]
        #   X = [X Y Z 1] 3D point in camera frame

        # Given a 3D point [X Y Z]', the projection (x, y) of the point onto the rectified image is given by:
        #  [u v w]' = P * [X Y Z 1]'
        #   x = u / w
        #   y = v / w

        # In order to match the convention, we need to append points with 1, so that we have X = [X Y Z 1]
        points_in_cam_frame = np.array(points_in_cam_frame)  # to numpy array

        # Make points_in_cam_frame has to have shape [X Y Z 1]
        ones = np.ones((len(points_in_cam_frame), 1), dtype=np.float32)
        points_in_cam_frame = np.append(points_in_cam_frame, ones, axis=1)

        # recive P, Projection matrix from CameraInfo
        P = camerainfo.P
        P = np.reshape(camerainfo.P, (3, 4))

        points_in_image = []
        for p in points_in_cam_frame:
            # Apply x = P * X
            p_in_image_plane = np.dot(P, p)
            u = p_in_image_plane[0]
            v = p_in_image_plane[1]
            w = p_in_image_plane[2]
            # Recall
            #  [u v w]' = P * [X Y Z 1]'
            #         x = u / w
            #         y = v / w
            x = u / w
            y = v / w
            points_in_image.append([x, y])

        # Draw these points to image plane
        idx = 0
        for p in points_in_image:
            try:
                # Draw a circle to represent this point
                # The color indicates ranges of the point
                cv2.circle(cv_image, (int(p[0]), int(p[1])), 3, 
                           int(50*ranges[idx]), thickness=3)
            except:
                pass
                #print("Cant project this point")
            idx += 1

        # Finally publish point projected image
        # We have achived on how one can represent Laser data in a camera image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main():

    rospy.init_node("SensorFuserNode")
    node = SensorFuserNode()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
