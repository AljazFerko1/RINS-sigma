#!/usr/bin/python3

import sys
import rospy
import dlib
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
import math

from os.path import dirname, join

#import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sklearn.cluster import KMeans
from std_msgs.msg import String
from task1.msg import Message
from glob import glob
import keras_vggface
from keras_vggface.vggface import VGGFace
from keras_vggface.utils import preprocess_input
from keras_vggface.utils import decode_predictions
import matplotlib.pyplot as plt
from PIL import Image as PILImage
import numpy as np
from mtcnn.mtcnn import MTCNN
from scipy.spatial.distance import cosine
from scipy.stats import wasserstein_distance


detected_points = []


class face_localizer:
    @staticmethod
    def read_images(path):
        path = path + '/*'
        images = {file.split("/")[-1].split(".")[0]:
                  plt.imread(file)
                  for file in glob(path)}
        return images

    def extract_face(self, img, required_size=(224, 224), detect_box=True):
        if not detect_box:
            face = img
            image = PILImage.fromarray(face)
            image = image.resize(required_size)
            face_array = np.asarray(image)
            return face_array
        print("Zacetek")
        results = self.detector.detect_faces(img)
        print("Konec")
        # extract the bounding box from the first face
        x1, y1, width, height = results[0]['box']
        x2, y2 = x1 + width, y1 + height
        # extract the face
        face = img[y1:y2, x1:x2]
        # resize pixels to the model size
        image = PILImage.fromarray(face)
        image = image.resize(required_size)
        face_array = np.asarray(image)

        return face_array

    def get_embeddings(self, faces):
        embeddings = {}
        for name, face in faces.items():
            sample = np.asarray(face, 'float32')
            sample = preprocess_input(sample, version=2)
            sample = np.asarray([sample])
            embeddings[name] = self.model.predict(sample)
        return embeddings

    def clossest_match(self, new_embeding, embedings):
        # calculate distance between embeddings
        distances = [(cosine(embeding.flatten(), new_embeding.flatten()), name)
                     for name, embeding in embedings.items()]
        return min(distances)[1]

    def face_recognition(self, rgb_image, pose):
        face = {"new": self.extract_face(rgb_image, detect_box=False)}
        new_embeding = self.get_embeddings(face)
        best_match = self.clossest_match(new_embeding["new"], self.embeddings)
        print(f"Best face match is {best_match}, at pose {pose}")
        self.face_poses[best_match] = pose
        #print(f"Current face_pose dict is:\n{self.face_poses}")

    def __post_init__(self):
        print("INITIALIZING FACE EMBEDDINGS...")
        images = self.read_images("/home/domen/ROS/src/task1/scripts/faces")
        faces = {name: self.extract_face(image)
                 for name, image in images.items()}
        self.embeddings = self.get_embeddings(faces)
        print(f"Finshed initializing")

    def __init__(self):
        rospy.init_node('face_localizer', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # The function for performin HOG face detection
        #self.face_detector = dlib.get_frontal_face_detector()
        protoPath = join(dirname(__file__), "deploy.prototxt.txt")
        modelPath = join(dirname(__file__),
                         "res10_300x300_ssd_iter_140000.caffemodel")

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
        self.markers_pub = rospy.Publisher(
            'face_markers', MarkerArray, queue_size=1000)
        self.pub = rospy.Publisher("face_position", Message, queue_size=10)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        self.detector = MTCNN()
        self.model = VGGFace(model='resnet50', include_top=False,
                             input_shape=(224, 224, 3), pooling='avg')
        self.face_poses = {}
        self.__post_init__()

    def get_pose(self, coords, dist, stamp):
        # Calculate the position of the detected face

        k_f = 554  # kinect focal length in pixels

        x1, x2, y1, y2 = coords

        face_x = self.dims[1] / 2 - (x1+x2)/2.
        face_y = self.dims[0] / 2 - (y1+y2)/2.

        angle_to_target = np.arctan2(face_x, k_f)

        # Get the angles in the base_link relative coordinate system
        x, y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        # Define a stamped message for transformation - directly in "base_link"
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
            rgb_image_message = rospy.wait_for_message(
                "/camera/rgb/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        try:
            depth_image_message = rospy.wait_for_message(
                "/camera/depth/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        # Convert the images into a OpenCV (numpy) format

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_image_message, "32FC1")
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
        blob = cv2.dnn.blobFromImage(cv2.resize(
            rgb_image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.face_net.setInput(blob)
        face_detections = self.face_net.forward()

        poses = 0

        for i in range(0, face_detections.shape[2]):
            confidence = face_detections[0, 0, i, 2]
            if confidence > 0.6:
                box = face_detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                box = box.astype('int')
                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]

                # Extract region containing face
                face_region = rgb_image[y1:y2, x1:x2]

                # Visualize the extracted face
                #cv2.imshow("ImWindow", face_region)
                # cv2.waitKey(1)

                # Find the distance to the detected face
                face_distance = float(np.nanmean(depth_image[y1:y2, x1:x2]))

                # Get the time that the depth image was recieved
                depth_time = depth_image_message.header.stamp

                # Find the location of the detected face
                pose = self.get_pose(
                    (x1, x2, y1, y2), face_distance, depth_time)

                if pose is not None and face_distance < 1.75:
                    print('Distance to face', face_distance)
                    """c = False
                    
                    for dp in detected_points:
                        if pose.position.x > dp[0] - 0.1 and pose.position.x < dp[0] + 0.1\
                            and pose.position.y > dp[1] - 0.1 and pose.position.y < dp[1] + 0.1:
                            c = True
                    
                    if c == False:
                        detected_points.append((pose.position.x, pose.position.y, pose.position.z))"""

                    #detected_points.append((pose.position.x, pose.position.y, pose.position.z))

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

                        # print(pose)
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
                        
                        print("Calling face recognition....")
                        self.face_recognition(face_region, pose)

    def depth_callback(self, data):

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Do the necessairy conversion so we can visuzalize it in OpenCV

        image_1 = depth_image / np.nanmax(depth_image)
        image_1 = image_1*255

        image_viz = np.array(image_1, dtype=np.uint8)

        #cv2.imshow("Depth window", image_viz)
        # cv2.waitKey(1)

        # plt.imshow(depth_image)
        # plt.show()


def main():

    face_finder = face_localizer()

    #rospy.Subscriber("chatter", String, callback)

    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        face_finder.find_faces()
        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
