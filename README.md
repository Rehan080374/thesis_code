# thesis_code
contains all the code required for thesis, additiol supporting liberaries required are libfranka and franka_ros for franka emika panda robot
Abstract
As collaborative robots (cobots for short) become more prominent in industrial settings,
intuitive ways of interaction between humans and cobots are essential for their success. In this
Thesis, a novel framework that enables a human to easily guide a cobot to co-manipulate an
extended object is presented. The developed framework enables a human to guide the object
grasped by the cobot in all dimensions of motion (translation, rotation, and gripper actuation).
To achieve this, a novel control has been developed that uses hand gestures and spatially-aware
features of the operator (e.g. operator’s location in space) with respect to the manipulated
object and generates object-centric control commands. The generated object-centric control
commands are then sent to an impedance controller for smooth object co-manipulation. Hand
gestures are recognized by a deep learning model using vision data and human position estima-
tion is calculated using stereocamera. The proposed framework is implemented on a 7-degrees
of freedom cobot equipped with a camera sensor which is used to estimate the operator’s posi-
tion with respect to the robot. Additionally, a wearable camera is worn by the human teammate
that monitors hand gestures. To evaluate the performance and usability of the proposed frame-
work, an experimental scenario is developed where a human is required to co-manipulate an
extended object by guiding the robot to pick it up from the table and place it into four tubes
with different orientations in different locations of the workspace. A user study of 16 partic-
ipants was conducted and the results are presented in this Thesis. The developed system was
compared to the state-of-the-art motion capture system and had an average error of 0.01 m
which is acceptable for our application. Moreover, the system was evaluated positively by the
participants in the study
