# CarND-Capstone Project
Students: 
- HM: audi-employee116@udacityenterprise.com
- TD: audi-employee120@udacityenterprise.com
- SB: audi-employee123@udacityenterprise.com

[motion_model]: ./docs/motion_model.png
[velocity_planning]: ./docs/velocity_planning.png
[controller_long]: ./docs/ControllerLong.png
[identification]: ./docs/driving_log_acc_cost_brake.png

[inference_eval]: ./docs/ssdv2tl_simrealbosch-eval.png
[inference_precision]: ./docs/ssdv2tl_simrealbosch-precision.png

[srb_lot]: ./imgs/test_tl_detector/inc_v2_coco_srb/left0000.png
[srb_sim]: ./imgs/test_tl_detector/inc_v2_coco_srb/left0003.png
[real_lot]: ./imgs/test_tl_detector/inc_v2_coco_real/left0000.png
[real_sim]: ./imgs/test_tl_detector/inc_v2_coco_real/left0003.png

[sim_lot]: ./imgs/test_tl_detector/inc_v2_coco_sim/left0000.png
[sim_sim]: ./imgs/test_tl_detector/inc_v2_coco_sim/left0003.png

Some initial notes before you deploy the project:
- Due to the terrible performance of the simulators camera-interface we had to add a lot of tweaks. 
- Not even the Udacity-Workspace is running properly with the camera activated. 
- It really is the simulator + interface! Even when we do not subscribe to the camera-topic the same lag can be observed!
- Especially if there are lot's of trees in the vicinity the simulator will take up all CPU resources and the drive-by-wire will spin out of control.
- The frame-rate for the inference was limited to `LIMIT_CAMERA_FPS = 4` (see `tl_detector.py`) in order to free up some CPU-resources. You might want to increase this on your PC.
- The `waypoint_updater` will only accept the track-waypoints once. **It will stop at the end of the waypoint-list.** We considered this to be the safe choice and desired behaviour?! 


There are several options available for setting up the development environment: 
- **Windows Host + VBox:** With the simulator running in Windows and ROS running in a Ubuntu VirtualBox. This option was *discarded due to terrible performance*. 
- **Ubuntu Host:** With the simulator and ROS running in the same OS. Better performance but still some lag with active camera. Difficult to set up the correct (old) versions and dependencies of *Carla*. Used during development.   
- **Project Workspace:** Webbased Workspace with a 50h limit supplied by Udacity. Will be used for testing compatibility with Carla. Camera can't be used due to terrible-lag.

## Setting up the Ubuntu Host
- Stick to the tested versions: Ubuntu 16.04 with ROS Kinetic 
- How To install Ubuntu alongside Win10: https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/

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

We need to make our python files and shell-scripts executable!

`find /home/YOURUSERNAME/CarND-Capstone/ -type f -iname "*.py" -exec chmod +x {} \;`

`find /home/YOURUSERNAME/CarND-Capstone/ -type f -iname "*.sh" -exec chmod +x {} \;`

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
- `update_distances()` pre-computes all distances between the given waypoints to avoid computational overhead during runtime.
- `closest_waypoint()` finds the closest waypoint to our current position. To speed things up the search starts from the last known waypoint if possible. The search is stopped once the waypoint-distances start increasing again.
- `next_waypoint()` find the next-waypoint ahead of us:
    - uses the math from the path-planning-project
    - transformation from quaternions to yaw-angle: https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
- `publish_final_waypoints()` Identifies stop-waypoints within our waypoint segment
    - If manual-driving is enabled, all target velocities will be set to the current velocity.
    - If automatic-driving is enabled, `path_to_trajectory()` will be used for velocity-planning.
- `path_to_trajectory()` Updates the velocities within the waypoint-segment.
    - A simple velocity profile with constant acceleration/deceleration is used!
    - A more sophisticated approach like quintic-polynomials does not make sense, because the `waypoint_follower` sends discontinuous velocities anyways ...
    - The math can be found in the image below. 
- We added the topic `'/current_waypoint'`: It publishes the closest-waypoint and avoids redundant calculations within the Traffic-Light-Detector.

![Math used for velocity planning][velocity_planning]

### Drive By Wire
This node is a pure velocity-controller. We subscribe to the current and desired linear- and angular-velocities. The target velocities themselves are calculated by the *waypoint_follower* cpp-node.
The following Python scripts are used in our implementation:
* `pid.py`: PID controller with anti-windup.
* `yaw_controller.py`: Unchanged, just inverse kinematics ...
* `twist_controller.py`: Initializes and updates all controllers. 
* `dbw_node.py`: The ROS wrapper 
    * Updates the controllers
    * In case of a positive torque request the actuation is converted to throttle. (power-factor calibrated by system-identification)
    * A minimum braking torque `CREEPING_TORQUE = 800` is applied while stopping (due to the creeping of the automatic transmission).
    * `RECORD_CSV = True`: A logfile with actuations, velocities and positions is written during manual-driving. This data may be used for system-identification.

#### System identification
Some data was recorded while accelerating, coasting and decelerating. We fitted a simple linear-motion model in order to get some unknown-parameters like the engine-power-factor. 
The basic idea of the identification is simple:

![Motion Model][motion_model]

The motion-model represents the simulator pretty well. Please note that the recorded acceleration is not used during the 
identification. But this way you get an impression of how noisy the velocity-data actually is.
At first it seemed weird that the brake-torque-factor is not equal to 1.0 as suggested by the Udacity-instructions. 
Upon further investigation the brake-torque-facor must be 1.0 instead of the identified 0.1144. 
The difference comes probably from a saturation of the brake-torque at ~2300Nm in the car simulator.

![Recorded vs. predicted][identification]

The identification was done using MATLAB: [mathscript](MATLAB/HM_SystemIdent.m) and this [dataset](MATLAB/driving_log_acc_coast_brake.csv).

#### Longitudinal controller
Because the `waypoint_follower` sends us discontinuous desired-velocities, we had to add a rate-limiter for the desired-velocities. 
The rate limiter uses the configured `max_acceleration` and `max_deceleration`. This way we can "reconstruct" the continuous velocity-profile from our `waypoint_updater`.

We implemented two types of controllers. The simple PID controller is activated by default because we expect a better robustness regarding the unknown model-parameters from *Carla*. 

1. The first control design (default) consists of a PID-controller that controls the velocity by adapting the total wheel torque.

2. The second longitudinal control design consists of:
    - PI-controller for velocity (acceleration as output)
    - Desired acceleration, as well as its gradient (jerk) are limited
    - Two degree of freedom controller for acceleration:
        - Feedforward control of the acceleration with identified longitudinal dynamics (see above)
        - PI controller for deviation between desired acceleration and actual acceleration

![Longitudinal Controller][controller_long]

The second concept needs a good model of the car as well as the vehicle acceleration, which can only be computed from 
the given velocity. 
Due to these shortcomings we use the first controller concept.

### Traffic Light Detection
This node has to detect the state of traffic-lights based on camera-images. In case of a red traffic-light it has to identify the stop-waypoint closest to the stop-line.
It will publish these stop-waypoints, the `waypoint_updater` will then use this information for the trajectory-planning.

Our approach:
- Uses the *tensorflow object_detection API* 
- Transfer learning using the *ssd_inception_v2_coco* model from the model-zoo
- We combine three training data-sets:
    - Labeled simulator images
    - Labeled test-lot images (extracted from the ROSBAG)
    - Images from the Training Bosch-Dataset: https://hci.iwr.uni-heidelberg.de/
   
The most important methods and features of the `tl_detector`:
- `find_stop_waypoint()` will find the optimal stop-waypoints for each traffic light, also considering the vehicle length. 
- `next_traffic_light()` finds the upcoming traffic-light and handles some tolerances necessary for safe stopping.
- `image_cb()` retrieves a new camera image
    - The frame-rate will be limited to `LIMIT_CAMERA_FPS = 4` you might increase this on a more potent PC. Inference takes 18ms on the CPU on our best machine ...
    - The Duty-Cycle will be kept below `MAX_DUTY_CYCLE = 0.75`
    - The `tl_classifier.py` is called for inference. There a tensorflow session is kept open to speed things up. 
- `current_waypoint_cb()` 
    - Directly gives us the current car-waypoint to avoid overhead.
    - If `FORCE_RED_LIGHT_SECONDS` is set, the car will stop at each stop-line regardless of the traffic-light-state (used for debugging). 

#### Tutorial for Training
Please note:
- Data-preparation is done in the `tl_data_preparation` folder (`create_tfrecords.ipynb`) and training is done in the `research` folder of the *Object-Detection-Sources*. 
- Only the frozen graph is copied to the ROS project: `./ros/src/tl_detector/light_classification/models_frozen/*`

Make sure that you have access to CUDA capable GPU (at least for the actual training):
```
import tensorflow as tf
tf.test.is_gpu_available()
# pray for return value: True
```

The first two links are a guide of how to set up your environment and your training-pipeline for Traffic-Light-Classification, some additional helpful links follow:
- [Tutorial for traffic light classifier](https://becominghuman.ai/traffic-light-detection-tensorflow-api-c75fdbadac62): This guides us through the training and evaluation!
- [Source code for traffic light classifier](https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI): The source code belonging to the guide above.
    - [Tensorflow sources](https://github.com/tensorflow/tensorflow) Tensorflow sources are required for the *object detection API*
    - [Install object_detection](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md) The *object detection API* has to be checked out into `../tensorflow/models/`
    - [Tensorflow model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md) Here we can download trained inference models
    - [LabelImg Tool](https://github.com/tzutalin/labelImg) A GUI for labeling additional images (boxes + labels)

#### Training Data
In order to merge our datasets (simulator, lot and Bosch) we have to do some pre-processing, have a look at [create_tfrecords.ipynb](tl_data_preparation/create_tfrecords.ipynb)
- The submission does not include any training-data due to its size. (The Bosch Dataset has about 17GB). 
- In the [data](tl_data_preparation/data) folder you will find directories for the training-sets. Each directory contains an `*info.txt` with download links.
    
#### Training and freezing of the inference model:

**PLEASE NOTE:** Different versions of the *object detection API* use different scripts for training. The shell-script [training_srb.sh](tl_training/training_srb.sh) has to be changed accordingly:
- `model_main.py` for newer *tensorflow* versions like v1.12 
- The older versions like *tensorflow* v1.3 on *Carla* use `train.py`.

Should be done on a powerful GPU-enabled machine, like AWS:
- Probably activate the virtual environment, Python2 makes less trouble with object-detection: `source ./venv2/bin/activate`
- All training is done within the research directory of the tensorflow installation `cd ~/tools/tensorflow/model/research`
- Add required `PYTHONPATH` by executing: ```export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim:`pwd`/object_detection```
- Start the training: `./training_srb.sh`
- Monitor training process, start tensorboard from a 2nd shell: `tensorboard --logdir=models/ssdv2tl_srb/train --port=8080`
- Connect with Webbrowser and ... wait ... for the first model checkpoint and evaluation run (probably 15min)
- Freeze the final model: `python object_detection/export_inference_graph.py --pipeline_config_path=models/ssdv2tl_srb_v1_3/pipeline.config --trained_checkpoint_prefix=models/ssdv2tl_srb_v1_3/train_0/model.ckpt-10000 --output_directory=models/ssdv2tl_srb_v1_3/frozen/`

Here some impressions from the *tensorboard* during training:

![Tensorboard finished training][inference_eval]

![Tensorboard final precision][inference_precision]

#### Image extraction
- **Test lot**: We can extract images from the ROSBAG by starting `roscore` and launching `roslaunch tl_detector/launch/tl_test.launch` 
- **Simulator**: We added a flag `SAVE_CAMERA_IMAGES_TO = None  # '/home/USER/CarND-Capstone/data/tl_test_simulator'` within `tl_detector.py`.

**Compatibility to Carla** (tensorflow==1.3) was accomplished using the following configuration:
- https://github.com/tensorflow/tensorflow.git (Branch v1.3.1)
- https://github.com/tensorflow/models.git (Branch r1.6.0)
- https://github.com/google/protobuf/releases/download/v3.0.0/protoc-3.0.0-linux-x86_64.zip (r1.5)

Separate models (Inception V2 Coco) for simulator (frozen_sim_tf1-3.pb) and test-lot (frozen_real_tf1-3.pb) as well as a generic model (frozen_srb_simon_tf1-3.pb) for both simulator and test lot were trained on corresponding training data.
The generic model performed sufficiently good so that it was used in both configurations: 

```
class TLClassifier(object):
    def __init__(self, is_site):
        # load classifier
        if is_site:
            model = 'models_frozen/ssdv2tl_srb/frozen_srb_simon_tf1-3.pb'
        else:
            model = 'models_frozen/ssdv2tl_srb/frozen_srb_simon_tf1-3.pb'
```

#### Test images
Images from the ROSBAG were used for testing the inference in the real-world-scenario. The next sections show the inference results of our three trained models. 
For additional test images have a look at [imgs/test_tl_detector](imgs/test_tl_detector).

##### Classificator (SSD Inception V2 Coco) trained on real and simulated image data (bosch + lot + sim)
The one used for simulator and test lot:

![universal model on lot image][srb_lot]

![universal model on sim image][srb_sim]

##### Classificator (SSD Inception V2 Coco) trained on real image data (bosch + lot)
Not used in our code!

![real model on lot image][real_lot]

![real model on sim image][real_sim]

##### Classificator (SSD Inception V2 Coco) trained on simulated image data (sim)
Not used in our code!

![simulator model on lot image][sim_lot]

![simulator model on sim image][sim_sim]




