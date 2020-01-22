# Power_Line_Navigation

Autonomous navigation, for inspection of electrical transmission networks, using only computer vision to correct trajectory.

## Requirements:

Tensorflow and ROS

This guide targets Ubuntu 16.04 and ROS Kinetic

## Steps:

To run Default SSD (Single Shot Detection) algorithm:

1. Install ROS: http://wiki.ros.org/kinetic/Installation/Ubuntu

2. Install camera dependencies

```
sudo apt-get install ros-kinetic-usb-cam
```

3. Install tensorflow into python virtualenv: https://www.tensorflow.org/install/install_linux or [virtualenv](https://pythonacademy.com.br/blog/python-e-virtualenv-como-programar-em-ambientes-virtuais)

```
sudo apt-get install python-pip python-dev python-virtualenv

virtualenv --system-site-packages ~/tensorflow

source ~/tensorflow/bin/activate

easy_install -U pip

pip install --upgrade tensorflow
```

4. Create folder and src

```
mkdir ~/navigation_ws/ && mkdir ~/navigation_ws/src/
```

5. Clone standard Vision messages repository and this repository into `catkin_ws/src`:

```
cd ~/navigation_ws/src

git clone https://github.com/Kukanani/vision_msgs.git

git clone https://github.com/osrf/tensorflow_object_detector.git
```

6. Build tensorflow_object_detector and Vision message

```
cd ~/navigation_ws && catkin build
```

7. Source catkin workspace's setup.bash:

```
source ~/navigation_ws/devel/setup.bash

source ~/tensorflow/bin/activate
```

> **Note:** This command needs to run from every new terminal you start. some questions can be easily taken from the links [stackoverflow](https://stackoverflow.com/questions/57614436/od-graph-def-tf-graphdef-attributeerror-module-tensorflow-has-no-attribut). Take care if all object detection notebooks and models have not been verified with TF 2.0. [Tensorflow](https://github.com/tensorflow/models/issues/7703)

```
 python -m pip install tensorflow==1.15
```

8. Run launch

```
roslaunch tensorflow_object_detector usb_cam_detector.launch

roslaunch tensorflow_object_detector drone_detector.launch

roslaunch tensorflow_object_detector rn_victor.launch
```

## If you want to try any other ML model:

1. Download any Object Detection Models from the Tensorflow Object detection API and place it in `data/models/`. You can find the models in tensorflow Object Detection Model Zoo: [Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md). Extract the `tar.gz` file.

2. Edit the MODEL_NAME and LABEL_NAME in detect_ros.py. By default it is `ssd_mobilenet_v1_coco_11_06_2017` with `mscoco_label_map.pbtxt` respectively.


3. 
```
$ python3
Python 3.4.1 (default, May 19 2014, 17:23:49) 
[GCC 4.9.0 20140507 (prerelease)] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import pickle
>>> pickle.dumps([1, 2, 'abc'], 2)
b'\x80\x02]q\x00(K\x01K\x02X\x03\x00\x00\x00abcq\x01e.'
>>> 
$ python 
Python 2.7.8 (default, Jul  1 2014, 17:30:21) 
[GCC 4.9.0 20140604 (prerelease)] on linux2
Type "help", "copyright", "credits" or "license" for more information.
>>> import cPickle
>>> cPickle.loads('\x80\x02]q\x00(K\x01K\x02X\x03\x00\x00\x00abcq\x01e.')
[1, 2, u'abc']
```

