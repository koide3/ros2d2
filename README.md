# ros2d2
*ros2d2* is a ROS package to make R2D2-like sounds. It is based on the following libraries:
- <a href="http://kevinboone.net/README_r2d2-voice.html">r2d2-voice</a> to synthesize prametric beep sounds (like whistle)
- <a href="https://pypi.python.org/pypi/ttastromech">ttastromech</a> to transform texts into R2D2 sounds

![R2D2](https://lumiere-a.akamaihd.net/v1/images/r2-d2_41dacaa9_68566da1.jpeg "R2D2")

## Installation

```bash
pip install ttastromech

cd ~/catkin_ws/src
git clone https://github.com/koide3/ros2d2.git

cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```

You may need to install libasound2-dev:
```bash
sudo apt install libasound2-dev
```

## Usage
```bash
roslaunch ros2d2 ros2d2.launch
```

```bash
# play a preset beep sound
$ rostopic pub /ros2d2_node/preset std_msgs/String "data: 'whistle'"

# generate R2D2 sound from a text
$ rostopic pub /ros2d2_node/speak std_msgs/String "data: 'hello world'"
```
Audio: [whistle](data/sound/whistle.ogg), [hello world](data/sound/hello.ogg)

## Topics
#### /ros2d2_node/preset
You can play preset beep sounds synthesized by *r2d2-voice*. Default preset sounds are; whistle, tonedown, toneup, failure, alert, confuse, and calculate. They are defined by a csv file at *ros2d2/data/presets*. You can add new sounds by adding commands to the file.

#### /ros2d2_node/speak
With this topic, you can play R2D2 voice transformed from a text using *ttastromech*. It makes sound by translating each character in the text to corresponding predefined R2D2 sound. 

#### /ros2d2_node/cmd_synth
This is a topic to make sound by directly giving commands to *r2d2-voice*. See <a href="http://kevinboone.net/README_r2d2-voice.html">r2d2-voice</a> for the details of the command.

## License

This package is released under GPLv3 as r2d2-voice adopts this license.
