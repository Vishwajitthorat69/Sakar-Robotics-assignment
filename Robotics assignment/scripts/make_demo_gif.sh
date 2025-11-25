#!/bin/bash
# Record a short ROS/Gazebo demo and convert to GIF (requires ffmpeg, rostest tools)
echo 'Start recording demo (you must run ros2 launch in another terminal)'
sleep 2
ffmpeg -y -f x11grab -video_size 1280x720 -i $DISPLAY -t 10 demo.mp4
ffmpeg -y -i demo.mp4 -vf 'fps=10,scale=640:-1:flags=lanczos' -loop 0 demo.gif
echo 'Created demo.gif'