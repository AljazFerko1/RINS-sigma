#!/usr/bin/python3

import sys
import rospy
import dlib
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import math
from matplotlib import pyplot as plt
import face_recognition

from os.path import dirname, join

#import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose, PoseWithCovarianceStamped
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sklearn.cluster import KMeans
from std_msgs.msg import String
from task1.msg import Message


detected_points = []
last_feedback = PoseWithCovarianceStamped()

class face_localizer:
    def __init__(self):
        rospy.init_node('face_localizer', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # The function for performin HOG face detection
        #self.face_detector = dlib.get_frontal_face_detector()
        protoPath = join(dirname(__file__), "deploy.prototxt.txt")
        modelPath = join(dirname(__file__), "res10_300x300_ssd_iter_140000.caffemodel")

        self.face_net = cv2.dnn.readNetFromCaffe(protoPath, modelPath)

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for showing markers in Rviz
        #self.markers = []
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Subscribe to the image and/or depth topic
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        # self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        # Publiser for the visualization markers
        self.markers_pub = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)
        self.pub = rospy.Publisher("task1_topic", Message, queue_size=10)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)


    def get_pose(self,coords,dist,stamp):
        # Calculate the position of the detected face

        k_f = 554 # kinect focal length in pixels

        x1, x2, y1, y2 = coords

        face_x = self.dims[1] / 2 - (x1+x2)/2.
        face_y = self.dims[0] / 2 - (y1+y2)/2.

        angle_to_target = np.arctan2(face_x,k_f)

        # Get the angles in the base_link relative coordinate system
        x, y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        ### Define a stamped message for transformation - directly in "base_link"
        #point_s = PointStamped()
        #point_s.point.x = x
        #point_s.point.y = y
        #point_s.point.z = 0.3
        #point_s.header.frame_id = "base_link"
        #point_s.header.stamp = rospy.Time(0)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp

        # Get the point in the "map" coordinate system
        try:
            point_world = self.tf_buf.transform(point_s, "map")

            # Create a Pose object with the same position
            pose = Pose()
            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z
        except Exception as e:
            print(e)
            pose = None

        return pose
    

    def find_faces(self):
        
        print('I got a new image!')

        # Get the next rgb and depth images that are posted from the camera
        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        try:
            depth_image_message = rospy.wait_for_message("/camera/depth_registered/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        # Convert the images into a OpenCV (numpy) format

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
            rgb_image = rgb_image[:,0:-100]
        except CvBridgeError as e:
            print(e)

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
            depth_image = depth_image[:,0:-100]
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        self.dims = rgb_image.shape
        h = self.dims[0]
        w = self.dims[1]

        # Tranform image to gayscale
        #gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        # Do histogram equlization
        #img = cv2.equalizeHist(gray)

        # Detect the faces in the image
        #face_rectangles = self.face_detector(rgb_image, 0)
        blob = cv2.dnn.blobFromImage(cv2.resize(rgb_image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.face_net.setInput(blob)
        face_detections = self.face_net.forward()
        
        poses = 0

        for i in range(0, face_detections.shape[2]):
            confidence = face_detections[0, 0, i, 2]
            if confidence>0.5:
                box = face_detections[0,0,i,3:7] * np.array([w,h,w,h])
                #box = face_detections[0,0,i,3:7] * np.array([w-100,h,w-100,h])
                box = box.astype('int')
                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]

                # Extract region containing face
                face_region = rgb_image[y1:y2, x1:x2]

                # Find the distance to the detected face
                face_distance = float(np.nanmean(depth_image[y1:y2,x1:x2]))

                # Visualize the extracted face
                """cv2.imshow("ImWindow", face_region)
                cv2.waitKey(1)"""

                # Get the time that the depth image was recieved
                depth_time = depth_image_message.header.stamp

                # Find the location of the detected face
                pose = self.get_pose((x1,x2,y1,y2), face_distance, depth_time)

                if pose is not None and face_distance > 200 and face_distance < 1200:
                    print('Distance to face', face_distance)
                    """c = False
                    
                    for dp in detected_points:
                        if pose.position.x > dp[0] - 0.1 and pose.position.x < dp[0] + 0.1\
                            and pose.position.y > dp[1] - 0.1 and pose.position.y < dp[1] + 0.1:
                            c = True
                    
                    if c == False:
                        detected_points.append((pose.position.x, pose.position.y, pose.position.z))"""
                        
                    #detected_points.append((pose.position.x, pose.position.y, pose.position.z))
                    
                    
                    
                    pose.position.x = (pose.position.x / 1000) + last_feedback.pose.pose.position.x
                    pose.position.y = (pose.position.y / 1000) + last_feedback.pose.pose.position.y
                    
                    c = False
                    for m in self.marker_array.markers:
                        if abs(pose.position.x - m.pose.position.x) < 0.5 and abs(pose.position.y - m.pose.position.y) < 0.5:
                            c = True
                            

                    if c == False:
                        
                        msg = Message()
                        msg.x = pose.position.x
                        msg.y = pose.position.y
                        msg.z = pose.position.z
                        self.pub.publish(msg)
                        
                        #print(pose)
                        # Create a marker used for visualization
                        self.marker_num += 1
                        marker = Marker()
                        marker.header.stamp = rospy.Time(0)
                        marker.header.frame_id = 'map'
                        marker.pose = pose
                        marker.type = Marker.CUBE
                        marker.action = Marker.ADD
                        marker.frame_locked = False
                        marker.lifetime = rospy.Duration.from_sec(0)
                        marker.id = self.marker_num
                        marker.scale = Vector3(0.1, 0.1, 0.1)
                        marker.color = ColorRGBA(0, 1, 0, 1)
                        self.marker_array.markers.append(marker)

                        self.markers_pub.publish(self.marker_array)
                          

    def depth_callback(self,data):

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Do the necessairy conversion so we can visuzalize it in OpenCV
        
        image_1 = depth_image / np.nanmax(depth_image)
        image_1 = image_1*255
        
        image_viz = np.array(image_1, dtype=np.uint8)

        cv2.imshow("Depth window", image_viz)
        cv2.waitKey(1)

        plt.imshow(depth_image)
        plt.show()

"""check = ""

def clustering():
    arr = np.array(detected_points)
    new_arr = []
    
    a = True
    for i in range(len(arr)):
        for j in range(len(arr[0])):
            if math.isnan(arr[i][j]):
                a = False
        if a:
            new_arr.append(arr[i])
        a = True
    
    new_arr = np.array(new_arr)
    
    kmeans = KMeans(n_clusters=5, n_init=30, random_state=0).fit(new_arr)
    
    markers_pub = rospy.Publisher('face_markers', MarkerArray, queue_size=5)
    marker_array = MarkerArray()
    
    marker = Marker()
    marker.id = 0
    marker.header.frame_id = 'map'
    marker.action = Marker.DELETEALL
    marker_array.markers.append(marker)
    markers_pub.publish(marker_array)
    rospy.sleep(0.2)
    
    #print(kmeans.cluster_centers_)
    
    for i in range(len(kmeans.cluster_centers_)):
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose.position.x = kmeans.cluster_centers_[i][0]
        marker.pose.position.y = kmeans.cluster_centers_[i][1]
        marker.pose.position.z = kmeans.cluster_centers_[i][2]
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.id = i + 1
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(0, 1, 0, 1)
        marker_array.markers.append(marker)
        markers_pub.publish(marker_array)   

def callback(data):
    global check
    check = data.data
    clustering()"""
    
def add_feedback(msg):
    global last_feedback
    last_feedback = msg 
    

def main():

    face_finder = face_localizer()
    
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, add_feedback)

    rate = rospy.Rate(1)
    rospy.sleep(3)
    while not rospy.is_shutdown():
        face_finder.find_faces()
        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
