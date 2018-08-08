To use pre-recorded videos, use video_stream_opencv. 


rosrun usb_cam usb_cam_node 
to run camera!

rosrun cannylive cannySender
to run the program
rosrun cannylive testSender
to see example code of a listener.

rostopic echo /errorDistances
to listen to the output.

there is a define in main.cpp where 0 is non-debug mode while 1 starts the debug mode.




Program will sleep until a camera is found on the topic. It does not turn off!

