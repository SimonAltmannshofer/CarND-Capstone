# CarND-Capstone Project
Students: 
* HM: 
* TD: 
* SB: 

[motion_model]: ./docs/motion_model.png

There are several options available for setting up the development environment: 
* **Windows Host + VBox:** With the simulator running in Windows and ROS running in a Ubuntu VirtualBox. This option was **discarded due to terrible performance**. 
* **Ubuntu Host:** With the simulator and ROS running in the same OS. Better performance but still some lag with active camera. Difficult to set up the correct (old) versions and dependencies of *Carla*. Used during development.   
* **Project Workspace:** Webbased Workspace with a 50h limit supplied by Udacity. Will be used for testing compatibility with Carla. 

## Setting up the Ubuntu Host
We will install Ubuntu next to Win10 as 2nd operating system. Once Ubuntu is installed we will set-up our Python development environment (PyCharm) and install ROS.
 
* HowTo: https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/
* Stick to the tested versions: Ubuntu 16.04 with ROS Kinetic 

### Install ROS: Robot Operating System
There is a nice guide for the ROS installation for Ubuntu: http://wiki.ros.org/kinetic/Installation/Ubuntu
It pretty much boils down to the following commands:
```
    1  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    2  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    3  sudo apt-get update
    4  sudo apt-get install ros-kinetic-desktop-full
    5  sudo rosdep init
    6  rosdep update
    7  echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    8  source .bashrc 
    9  sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### Installing project dependencies
Additional dependency required for Carla and the Capstone Project, execute the following command:

`bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_mkz_ros/raw/default/dbw_mkz/scripts/sdk_update.bash)`

Clone the Udacity Project from Git

`git clone https://github.com/udacity/CarND-Capstone.git`

We need to make our python files executable!

`find /home/YOURUSERNAME/CarND-Capstone/ -type f -iname "*.py" -exec chmod +x {} \;`

###  Running the ROS project
```
cd ~/CarND-Capstone/ros/
catkin_make
source devel/setup.bash 
roslaunch launch/styx.launch 
```
Note that a recompile is not necessary if you only changed Python code.

### IDE setup
You can use the *Ubuntu Software* Center to install **PyCharm CE**.  
* Create a new project and select the existing CarND-Capstone folder. (Confirm that you want to create the project based on existing sources)
* Make sure that you select the appropriate Python interpreter. 

## Project Development
The development was done in the suggested order, some notes regarding the implementation follow.

### Waypoint Updater 
This node extracts a given number of waypoints ahead of our current location. The velocities of these waypoints are updated in order to get a proper trajectory (time dependant).
* `update_distances()` pre-computes all distances between the given waypoints to avoid computational overhead during runtime.
* `closest_waypoint()` finds the closest waypoint to our current position. To speed things up the search starts from the last known waypoint if possible. The search is stopped once the waypoint-distances start increasing again.
* `next_waypoint()` find the next-waypoint ahead of us:
    * uses the math from the path-planning-project
    * transformation from quaternions to yaw-angle: https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
* `publish_final_waypoints()` To speed things up we only publish upon changes. Identifies stop-waypoints within our waypoint segment.
* `path_to_trajectory()` Updates the velocities within the waypoint-segment.
    * A simple velocity profile with constant acceleration/deceleration is used!
    * `v_traj = math.sqrt(2*dist_rem*PLAN_ACCELERATION)`
* We added the topic `'/current_waypoint'`: It publishes the closest-waypoint and avoids redundant calculations within the Traffic-Light-Detector.

Possible Improvements:
* Velocity Planning with less jerk, e.g. quintric-poly ...
* Generation of additional (computed) waypoints instead of the fixed tolerances: `NUM_WP_STOP_AFTER_STOPLINE` and `NUM_WP_STOP_BEFORE_STOPLINE`.
* Should we do our velocity planning based on the config values of maximum acc/deceleration?

### Drive By Wire
This node is a pure velocity-controller. We subscribe to the current and desired linear- and angular-velocities. The target velocities themselves are calculated by the dark-magic of the *waypoint_follower* cpp-node.
* `pid.py`: This simple PID controller is used for the throttle-control (normalized 0.0 to 1.0). I added a proper anti-windup.
* `yaw_controller.py`: Unchanged, just inverse kinematics ...
* `twist_controller.py`: Initializes and updates all controllers. 
* `dbw_node.py`: The ROS wrapper 
    * Updates the controllers
    * In case of negative throttle the actuation is converted to brake-torque in Nm.
    * A minimum braking torque `CREEPING_TORQUE = 700` is applied while stopping (due to the creeping of the automatic transmission).
    * `RECORD_MANUAL = True`: A logfile with actuations, velocities and positions is written during manual-driving. This data may be used for system-identification.

```
# SIMULATOR: fitted coefficient between throttle and acceleration
acceleration = 10.84 * throttle
# mass * acceleration = force | force = torque / radius
brake = -acceleration * self.mass * self.wheel_radius
```

#### Longitudianl controller

The longitudinal controller consists of 
- PI-controller for velocity (acceleration as output)
- desired acceleration value as well as its gradient are limited
- two degree of freedom controller for acceleration
    - feedforward control of the acceleration with identified longitudinal dynamics (see below)
    - PI controller for deviation between desired acceleration and actual acceleration

#### Controller Tuning
To determine the power-ratio of the car a simple motion model was fitted to a recorded-dataset (manual driving).

![Motion Model][motion_model]

The coefficients of a time-discrete PID-controller were tuned using Matlab-Simulink. Tuned to be fast (we follow a continuous trajectory) and robust (the real car might behave very different ...)
```
# tuned PID controller for the throttle
self.throttle_ctr = PID(0.292, 0.2552, 0.02631, decel_limit, accel_limit)
```



Possible Improvements:
* Identification of a proper motion model, but this might not help a lot with the real car
* Torque feed forward control including steering
* Check if calculations between torque, throttle and braking are valid and consistent

### Object Detection
After some research this is my (initial) approach:
- Using the _tensorflow object_detection API_ 
- Transfer learning using the _ssd_inception_v2_coco_ model from the model-zoo
- We combine three training data-sets:
    - Labeled simulator images
    - Labeled test-lot images (extracted from the ROSBAG)
    - Images from the Bosch-Dataset: https://hci.iwr.uni-heidelberg.de/

**PLEASE NOTE:** 
- All data-preparation and training is done in the `research` folder of the *Object-Detection-Sources*. 
- Only the frozen graph is copied to the ROS project: `./ros/src/tl_detector/light_classification/models_frozen/*`
    - ssdv2tl .. was trained with simulator data only (not used)
    - ssdv2tl_srb ... was trained with all three datasets (used for both, simulator and test-lot)
- Both models were created with Tensorflow 1.12 which is too new for *Carla*. 
- WE HAVE TO DO THE TRAINING AGAIN WITH TENSORFLOW 1.3 (probably in the Udacity-Project-Workspace)

Make sure that you have access to CUDA capable GPU (at least for the actual training):
```
import tensorflow as tf
tf.test.is_gpu_available()
# pray for return value: True
```

The first two links are a guide of how to set up your environment and your training-pipeline for Traffic-Light-Classification, some additional helpful links follow:
- [Tutorial for traffic light classifier](https://becominghuman.ai/traffic-light-detection-tensorflow-api-c75fdbadac62): This guides us through the training and evaluation!
- [Source code for traffic light classifier](https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI): The source code belonging to the guide above.
- [Tensorflow sources](https://github.com/tensorflow/tensorflow)
- [Install object_detection](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md)
- [Tensorflow model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md)
- [Up to date object_detection pipeline configuration](https://github.com/developmentseed/label-maker/blob/94f1863945c47e1b69fe0d6d575caa0b42aa8d63/examples/utils/ssd_inception_v2_coco.config)
- [Running local training job](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/running_locally.md#running-locally)
- [LabelImg Tool](https://github.com/tzutalin/labelImg)
- [Howto TFRecords from VOC](https://github.com/tensorflow/models/blob/4f32535fe7040bb1e429ad0e3c948a492a89482d/research/object_detection/g3doc/preparing_inputs.md)


#### Training Data
In order to merge our datasets (simulator, lot and Bosch) we have to do some pre-processing, have a look at `create_tfrecords.ipynb`
* The Bosch Dataset is huge (about 17GB). I only included a small subset, that I converted to jpg. 
    * You should directly load the `train_good.yaml` instead of `train.yaml` and skip the according sections within the juypter notebook. 
    * Continue with `def get_bosch_record_with_relative_path`

#### Training and freezing of the inference model:

**PLEASE NOTE:** The current version of the training script `training_srb.sh` was written for the newer Object-Detection-API using `model_main.py`. The older versions use `train.py` like in the linked-blog-post-guide.

Should be done on a powerful GPU-enabled machine, like AWS:
- Probably activate the virtual environment, Python2 makes less trouble with object-detection: `source ./venv2/bin/activate`
- All training is done within the research directory of the tensorflow installation `cd ~/tools/tensorflow/model/research`
- Add required `PYTHONPATH` by executing: ```export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim:`pwd`/object_detection```
- Start the training: `./training_srb.sh`
- Monitor training process, start tensorboard from a 2nd shell: `tensorboard --logdir=models/ssdv2tl_srb/train --port=8080`
- Connect with Webbrowser and ... wait ... for the first model checkpoint and evaluation run (probably 15min)
- Freeze the final model: `python object_detection/export_inference_graph.py --pipeline_config_path=models/ssdv2tl_srb_v1_3/pipeline.config --trained_checkpoint_prefix=models/ssdv2tl_srb_v1_3/train_0/model.ckpt-10000 --output_directory=models/ssdv2tl_srb_v1_3/frozen/`

#### Image extraction
* **Test lot**: We can extract images from the ROSBAG by starting `roscore` and launching `roslaunch tl_detetor/launch/tl_test.launch` 
* **Simulator**: I added a flag `SAVE_CAMERA_IMAGES_TO = None  # '/home/USER/CarND-Capstone/data/tl_test_simulator'` within `tl_detector.py`.

Possible Improvements:
* Some additional training data from the simulator might be helpful (especially yellow and red)
* We have to get it compatible to Carla (tensorflow==1.3)
* Maybe different models for simualator and test-lot:

```
class TLClassifier(object):
    def __init__(self, is_site):
        # load classifier
        if is_site:
            model = 'models_frozen/ssdv2tl_srb/frozen_inference_graph.pb'
        else:
            model = 'models_frozen/ssdv2tl_srb/frozen_inference_graph.pb'
```

# BEST OF LUCK

# Original README


This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
