ghost_bridge
============
ghost_bridge is a ROS package that connects the Hanson Robotics stack and [GHOST](https://github.com/opencog/opencog/tree/master/opencog/ghost).
It provides a set of actions that can be called from GHOST and a set of perceptions that are sent to the OpenCog
Atomspace, an overview of the actions and perceptions are provided below.

#### Actions:
* **say**: make the robot vocalize text.
* **gaze-at**: turn the robot's eyes towards the given target point.
* **face-toward**: turn the robot's face towards the given target point.
* **blink**: set the robot's blink cycle.
* **saccade**: set the robot's eye saccade cycle, i.e. how the eye's twitch and move around automatically.
* **emote**: set the robot's emotional state.
* **gesture**: set a pose on the robot's face.
* **soma**: sets the robot's background facial expressions.

#### Perceptions:
* **perceive-emotion**: perceive an emotion.
* **perceive-eye_state**: perceive the state of a person's eyes.
* **perceive-face-talking**: the probability of whether a particular face is talking or not.
* **perceive-word**: perceive an individual word that is a part of the sentence a person is currently speaking.
* **perceive-sentence** (ghost): perceive the whole sentence after the user has finished speaking.

Setup
-------
Follow the steps below to setup the ghost_bridge stack.

#### 1. Setup HEAD stack
Setup the Hanson Robotics head stack by following [these instructions](https://github.com/hansonrobotics/hrtool).

#### 2. Configure ~/.bashrc
Add the following to your ~/.bashrc, 
```bash
source /opt/ros/kinetic/setup.bash
export HR_WORKSPACE="$(hr env | grep HR_WORKSPACE | cut -d = -f 2)"
source ${HR_WORKSPACE}/HEAD/devel/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda/lib64/
```

Ensure you source your ~/.bashrc afterwards:
```bash
source ~/.bashrc
```

#### 3. Install dependencies
##### 3.1 With NVIDIA GPU support
Ensure you have an NVIDIA binary driver installed. In Ubuntu, open *Additional Drivers* and select the latest NVIDIA 
binary driver that is available, e.g. *Using NVIDIA binary driver - version 396.26 from nvidia-396 (proprietary)*.

Install the Tensorflow GPU dependencies: *CUDA Toolkit 9.0* and *cuDNN SDK v7*. Follow the instructions [here](https://www.tensorflow.org/install/install_linux),
under the heading *NVIDIA requirements to run TensorFlow with GPU support*.

Add the following to your ~/.bashrc:
```bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda-9.0/lib64/
export CUDA_BIN_PATH=/usr/local/cuda-9.0
```

Ensure you source your ~/.bashrc afterwards:
```bash
source ~/.bashrc
```

Install Tensorflow GPU and Keras:
```bash
sudo pip install tensorflow-gpu==1.8.0 keras==2.1.6
```

Install dlib:
```bash
git clone https://github.com/davisking/dlib.git
cd dlib
sudo -E bash -c "python setup.py install"
```

##### 3.2 Without NVIDIA GPU support
Install the following:
```bash
sudo pip install dlib==19.13.0 tensorflow==1.8.0 keras==2.1.6
```

#### 4. Checkout ghost repos
Clone ghost_bridge and checkout v0.5.4:
```bash
cd ${HR_WORKSPACE}/HEAD/src && git clone https://github.com/opencog/ghost_bridge.git
cd ${HR_WORKSPACE}/HEAD/src/ghost_bridge && git fetch --all --tags --prune
git checkout tags/v0.5.6
```

Clone ros_people_model:
```bash
cd ${HR_WORKSPACE}/HEAD/src && git clone https://github.com/elggem/ros_people_model.git
cd ${HR_WORKSPACE}/HEAD/src/ros_people_model && git fetch --all --tags --prune
git checkout tags/v0.2.0
```

Checkout the ghost branch of chatbot, hr_launchpad and configs:
```bash
cd ${HR_WORKSPACE}/HEAD/src/chatbot && git checkout ghost
cd ${HR_WORKSPACE}/configs && git checkout ghost
cd ${HR_WORKSPACE}/hr_launchpad && git checkout ghost
```

Build the head stack
```bash
hr build head
```

#### 5. Setup OpenCog
Install octool and OpenCog dependencies:
```bash
sudo curl -L http://raw.github.com/opencog/ocpkg/master/ocpkg -o /usr/local/bin/octool
sudo chmod +x /usr/local/bin/octool
octool -d
```

Install OpenCog repos:
```bash
hr update opencog
```

Install relex dependencies:
```bash
cd ${HR_WORKSPACE}/OpenCog/relex/install-scripts && ./install-ubuntu-dependencies.sh
```

Remove ros-behavior-scripting and checkout the ghost-lai branch of opencog and atomspace:
```bash
sudo rm -r ${HR_WORKSPACE}/OpenCog/ros-behavior-scripting
cd ${HR_WORKSPACE}/OpenCog/atomspace && git checkout ghost-lai
cd ${HR_WORKSPACE}/OpenCog/opencog && git checkout ghost-lai
```

Build OpenCog:
```bash
hr clean opencog
hr build opencog
```

Running
-------
Run the robot first:
```bash
hr run --nogui --dev sophia10
```

Then run ghost_bridge:
```bash
rosrun ghost_bridge run.sh
```

To stop the robot:
```bash
hr stop
```

To stop ghost_bridge:
```bash
rosrun ghost_bridge stop.sh
```

Tips for running on the Zotac:
------------------------------
#### PCIe Bus Error
Run the command `dmesg`, if you get the error `PCIe Bus Error: severity=Corrected, type=Physical Layer, id=00e6(Receiver ID)`, then you can fix it by doing the following.

Run `sudo nano /etc/default/grub`

And change this line: `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"`

To: `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash pcie_aspm=off"`

Then run `sudo update-grub`

Design Goals
------------
TODO

Architecture
-------------------------------
TODO
