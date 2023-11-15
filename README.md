# TF ROS2 Publisher
A ROS2 package for publishing the transform of a TF. 

## Installation
Clone the repository in your workspace
```
cd <your_ws>/src
git clone https://github.com/Hydran00/tf_publisher_ROS2.git tf_publisher
```

## Define the link which you want to publish
You just have to look at the launch `launch/tf_publisher.launch.py` and set your target frame and the frame with respect to which you want to publish the rototransformation.

```
"ee_frame_name": tool_frame_name, "base_frame_name": base_frame_name, 'rate': str(rate), 'topic':topic
```