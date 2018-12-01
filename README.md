# Notes

## install pycharm
`find /home/workspace/your/directory -type f -iname "*.py" -exec chmod +x {} \;`

## install ros
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

## installing project dependencies
Additional dependency required for Carla and the Capstone Project

`bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_mkz_ros/raw/default/dbw_mkz/scripts/sdk_update.bash)`

Cloning the Udacity Project from Git

`git clone https://github.com/udacity/CarND-Capstone.git`

We need to make our python files executable!

`find /home/morieris/CarND-Capstone/ -type f -iname "*.py" -exec chmod +x {} \;`

## Object Detection
After some research this is my (initial) approach:
- Using the _tensorflow object_detection API_ 
- Transfer learning using the _ssd_inception_v2_coco_ model from the model-zoo


Some additional links and tutorials:
- [Tensorflow sources](https://github.com/tensorflow/tensorflow)
- [Tutorial for traffic light classifier](https://becominghuman.ai/traffic-light-detection-tensorflow-api-c75fdbadac62)
- [Source code for traffic light classifier](https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI)
- [Install object_detection](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md)
- [Tensorflow model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md)
- [Up to date object_detection pipeline configuration](https://github.com/developmentseed/label-maker/blob/94f1863945c47e1b69fe0d6d575caa0b42aa8d63/examples/utils/ssd_inception_v2_coco.config)
- [Running local training job](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/running_locally.md#running-locally)
- [LabelImg Tool](https://github.com/tzutalin/labelImg)
- [Howto TFRecords from VOC](https://github.com/tensorflow/models/blob/4f32535fe7040bb1e429ad0e3c948a492a89482d/research/object_detection/g3doc/preparing_inputs.md)

Actual training and freezing of the inference model:
- Probably activate the virtual environment, Python2 makes less trouble with object-detection: `source ./venv2/bin/activate`
- All training is done within the research directory of the tensorflow installation `cd ~/tools/tensorflow/model/research`
- Add required `PYTHONPATH` by executing: ```export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim:`pwd`/object_detection```
- Start the training: `./training.sh`
- Monitor training process, start tensorboard from a 2nd shell: `tensorboard --logdir=models/ssdv2tl/train` --port=8080
- Connect with Webbrowser and ... wait ...
- Freeze the final model: `python object_detection/export_inference_graph.py --pipeline_config_path=models/ssdv2tl/ssd_inception-traffic-udacity_sim_2018.config --trained_checkpoint_prefix=models/ssdv2tl/train/model.ckpt-5000 --output_directory=models/ssdv2tl/frozen/`

TODOs:
- Fit motion model to recorded position/velocity data 
- Optimize PID controller - or replace with pilot-control plus PID for remaining error
- Train traffic light detector for real-world images or both?! additional datasets available online
- Convert training data from yaml to VOC XML (easier TFRecord generation)


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
