#!/usr/bin/env python
## Author: Alan Tavares
## Date: August, 12, 2019
# Purpose: Ros node to detect objects using tensorflow
import os
import sys
import cv2
import rospy
import numpy as np
try:
    import tensorflow as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  sudo apt install python-pip")
    print("  sudo pip install tensorflow")
    sys.exit(1)

# ROS related imports
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Vector3

# Object detection module imports
import object_detection
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

# SET FRACTION OF GPU YOU WANT TO USE HERE
GPU_FRACTION = 0.0

######### Set model here ############
MODEL_NAME =  'modo_congelado'
# By default models are stored in data/models/
MODEL_PATH = os.path.join(os.path.dirname(sys.path[0]),'data','models' , MODEL_NAME)
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_PATH + '/frozen_inference_graph.pb'
######### Set the label map file here ###########
LABEL_NAME = 'mscoco_label_map.pbtxt'
# By default label maps are stored in data/labels/
PATH_TO_LABELS = os.path.join(os.path.dirname(sys.path[0]),'data','labels', LABEL_NAME)
######### Set the number of classes here #########
NUM_CLASSES = 1

detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`,
# we know that this corresponds to `airplane`.  Here we use internal utility functions,
# but anything that returns a dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Setting the GPU options to use fraction of gpu that has been set
config = tf.ConfigProto()
config.gpu_options.per_process_gpu_memory_fraction = GPU_FRACTION

# RCNN
class RCNN:

    def __init__(self):

        self.image_pub = rospy.Publisher("rcnn/debug_image/compressed",CompressedImage, queue_size=1)
        self.rcnn_pub = rospy.Publisher('rcnn/nav_position', Vector3, queue_size=1)

        # Create a supscriber from topic "image_raw"
        self.image_sub = rospy.Subscriber("bebop/image_raw/compressed", CompressedImage, self.image_callback, queue_size=1)
        self.sess = tf.Session(graph=detection_graph,config=config)

        self.DIAMETER_LANDMARCK_M = rospy.get_param('~markerSize_RCNN', 0.03)
        self.DISTANCE_FOCAL = rospy.get_param('~distance_focal', 740)
        self.MAX_NUMBER_OF_BOXES = rospy.get_param('~max_number_of_boxes', 6)
        self.MINIMUM_CONFIDENCE = rospy.get_param('~minimum_confidence', 0.95)

        self.VERBOSE = False

        rospy.loginfo("%s is %f (defaut)", rospy.resolve_name('~markerSize_RCNN'), self.DIAMETER_LANDMARCK_M)
        rospy.loginfo("%s is %f (defaut)", rospy.resolve_name('~distance_focal'), self.DISTANCE_FOCAL)
        rospy.loginfo("%s is %f (defaut)", rospy.resolve_name('~max_number_of_boxes'), self.MAX_NUMBER_OF_BOXES)
        rospy.loginfo("%s is %f (defaut)", rospy.resolve_name('~minimum_confidence'), self.MINIMUM_CONFIDENCE)

    def image_callback(self, data):

        np_arr = np.fromstring(data.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        
        #-- Print 'X' in the center of the camera
        image_height,image_width,channels = image.shape
        cv2.putText(image, "X", (image_width/2, image_height/2), font, 1, (0, 0, 255), 2, cv2.LINE_AA)

        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        image_np = np.asarray(image)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
        # Number of objects detected
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')

        (boxes, scores, classes, num) = self.sess.run([detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        objects=vis_util.visualize_boxes_and_labels_on_image_array(
            image,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            min_score_thresh=self.MINIMUM_CONFIDENCE,
            use_normalized_coordinates=True,
            line_thickness=5)

        msg_nav = Vector3()
        list_positivey = []
        list_negativey = []
        list_z = []

        # Object search
        if len(objects) > 0:
            for i in range(len(objects)):
                self.object_predict(objects[i],image_height,image_width,list_positivey,list_negativey,list_z)

            medz_ant = sum(list_z)/len(list_z)
            msg_nav.z = medz_ant

            # negativey == True
            if len(list_negativey) > 0:
                # mean of list x-
                medy_ant_n = sum(list_negativey)/len(list_negativey)
                # rospy.logdebug("negativey == True: %f", medy_ant_n)
                msg_nav.x = medy_ant_n

            # negativey == False and positivey == True
            elif len(list_positivey) > 0 and len(list_negativey) == 0:
                # mean of list x+
                medy_ant_p = sum(list_positivey)/len(list_positivey)
                # rospy.logdebug("negativey == False and positivey == True: %f", medy_ant_p)
                msg_nav.x = medy_ant_p
            else:
                msg_nav.x = 0
                # rospy.logdebug("negativey == False and positivey == False: %f", msg_nav.x)
                rospy.logdebug("--------------------------------")

        self.rcnn_pub.publish(msg_nav)
        # rospy.logdebug("Altura Filtrada (out): %f", msg_nav.z)
        # rospy.logdebug("--------------------------------")

        ########################################################################################

        #### Create CompressedIamge ####
        msg = CompressedImage()
        #msg.header.stamp = data_ros.header.stamp
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)


    def object_predict(self,object_data,img_height,img_width,list_positivey,list_negativey,list_z):

        dimensions=object_data[2]

        size_x = int((dimensions[3] - dimensions[1])*img_width)
        size_y = int((dimensions[2] - dimensions[0])*img_height)
        center_x = int((dimensions[1] + dimensions [3])*img_width/2)
        center_y = int((dimensions[0] + dimensions[2])*img_height/2)

        ###################################################################################

        pixelDiametro = size_x
        # choose the bigest size
        if(size_y > size_x):
            pixelDiametro = size_y

        ###################################################################################

        if self.VERBOSE == True:
            rospy.logdebug("size_x:      %f", size_x)
            rospy.logdebug("size_y:      %f", size_y)
            rospy.logdebug("center_x:    %f", center_x)
            rospy.logdebug("center_y:    %f", center_y)
            rospy.logdebug("--------------------------------------------")

        metersDiametroLandmarck = self.DIAMETER_LANDMARCK_M
        #metersDiametroLandmarck = 0.03

        distFocus_real = self.DISTANCE_FOCAL
        #distFocus_real = 740

        z_real = float((metersDiametroLandmarck * distFocus_real) / pixelDiametro)

        pixel_x = int((center_x-(img_width/2))*(-1))
        pixel_y = int((center_y-(img_height/2))*(1))

        ###################################################################################

        if self.VERBOSE == True:
            rospy.logdebug("Diametro Marcador Real (instante):  %f", metersDiametroLandmarck)
            rospy.logdebug("Distancia Focal Real:               %f", distFocus_real)
            rospy.logdebug("Diametro (pixel):                   %f", pixelDiametro)
            rospy.logdebug("Z Drone (real):                     %f (m)", z_real)
            rospy.logdebug("--------------------------------------------")

        ###################################################################################
        
        k = float(metersDiametroLandmarck/pixelDiametro)

        pointx = pixel_x*k
        pointy = pixel_y*k
        pointz = z_real
        
        if pointy > 0:
            list_positivey.append(pointy)
        else:
            list_negativey.append(pointy)

        list_z.append(pointz)

def main(args):
    rospy.init_node('rcnn_pose_node', log_level=rospy.DEBUG)
    rcnn_pose = RCNN()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    # cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)