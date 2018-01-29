#iPentant version 2.3
#2016-12-02 update
#update log

1. rewrite command "WAIT" related functions. now well work with inputs and outputs.
2. rewrite IO_device initial function. The initial variables are -1, so that they won't be misunderstood by clients when the IO_device is not connected.

#iPentant version 2.2
#2015-03-10 update
#update log

1. reconstruct Class Playback;
2. add playbackFromTo() function;
3. add child project function;
4. add new Struct Interpreter including 7 methods;
5. reconstruct Class Directory to Class Project and Class ProjectHandler;
6. move Class ROS_Communication initialization into Class MainWindow;
7. move the object of Struct State, Parameter, Controller_State into run() in Class ROS_Communication;
8. add new Struct Actor;
