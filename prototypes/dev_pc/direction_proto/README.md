# Direction Prototype

This project is to check if we can make a simple direction indicator.
Goal is to listen to the topic '/cmd_vel'.

## Setting up the prototype

Use https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#write-the-subscriber-node


## Publish a topic on the '/topic' topic
ros2 topic pub /topic std_msgs/msg/String "{data: Hello}"