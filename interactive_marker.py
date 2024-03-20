#!/usr/bin/env python
import sys
import rospy
import tf.transformations as transform
import tf.transformations
import numpy as np
import math
import actionlib
from franka_gripper.msg import GraspAction, GraspGoal,MoveAction,MoveGoal
from geometry_msgs.msg import Pose,PointStamped
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs
from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
from gtts import gTTS
from playsound import playsound
from sensor_msgs.msg import JointState 
from scipy.spatial.transform import Rotation as R
marker_pose = PoseStamped()
p_pose=PoseStamped()
R_pose=PoseStamped()
radius_pose=Pose()
initial_pose_found = False
# pose_pub = None
# origin_found=False
gesture_p='None'
gesture='None'
p_position="none"
previous_state="open"
gripper_c_state=''
gripper_p_state=''
# item_placed=False 
repeat=0
p_repeat=0
position="none"
radius=0
scale=1
mode="slow"
# pose2=[0,0,0,0,0]
# euler=[0,0,0]
# pose1=[0,0,0]
p=[0,0,0,0,0,0]
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]



def grasp_client():
    global gripper_state,gripper_p_state,gripper_c_state
    if gripper_state=="open":
        print("closing")
        client = actionlib.SimpleActionClient('franka_gripper/grasp',GraspAction)
        print('waiting for the action server to start')
        timeout=rospy.Duration(2)
        client.wait_for_server(timeout)
        goal = GraspGoal()
        goal.force = 50
        goal.epsilon.inner = 0.035
        goal.epsilon.outer = 0.0410
        goal.speed = 0.1
        goal.width = 0.04
        #print("goal=",goal)
        client.send_goal(goal)
        wait = client.wait_for_result(timeout)

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()
    else:   
        print("opening")  
        client = actionlib.SimpleActionClient('franka_gripper/move',MoveAction)
        print('waiting for the action server to start')
        gripper_p_state=gripper_state
        timeout=rospy.Duration(2)
        client.wait_for_server(timeout)
        goal = MoveGoal()
        goal.speed = 0.1
        goal.width = 0.08
        client.send_goal(goal)
        wait = client.wait_for_result(timeout)

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            gripper_c_state=gripper_state
        else:
            return client.get_result()
def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time(0)

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(2))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise
def gripper_state_callback(msg):
    global gripper_state
    g=msg.position
    
    if g[0]>0.035:
        gripper_state='open'
    else:
        gripper_state='close'   
    
    # print(gripper_state)         

def publisherCallback(msg, link_name):
    marker_pose.header.frame_id = link_name 
    marker_pose.header.stamp = rospy.Time(0)
    
    pose_pub.publish(marker_pose)    

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitcroll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians    
def get_quaternion_from_euler(roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
 
        return [qx, qy, qz, qw]


def gesture_callback(msg):  
    global  position,radius,gesture
    gesture=msg.data 
    # print("position at gesture callback is ",position)
    # gesture_control(position,gesture,radius)
    print("gesture = {}, radius = {}, position = {}".format(gesture,radius,position))
   
              
    
    


               
def franka_state_callback(msg):
    
    global initial_pose_found,p,euler,init_euler,p_pose
    # initial_pose_found=False
    initial_quaternion = \
        transform.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
    if initial_pose_found == False:
        marker_pose.pose.orientation.x = initial_quaternion[0]
        marker_pose.pose.orientation.y = initial_quaternion[1]
        marker_pose.pose.orientation.z = initial_quaternion[2]
        marker_pose.pose.orientation.w = initial_quaternion[3]
        marker_pose.pose.position.x = msg.O_T_EE[12]
        marker_pose.pose.position.y = msg.O_T_EE[13]
        marker_pose.pose.position.z = msg.O_T_EE[14]
        init_euler= transform.euler_from_matrix(np.transpose(np.reshape(msg.O_T_EE,
                                (4, 4))))
        initial_pose_found = True

    euler= transform.euler_from_matrix(np.transpose(np.reshape(msg.O_T_EE,
                                    (4, 4))))
    
    
    # print("initial x =" ,init_euler[0],"current x = ",euler[0])
    # p =[msg.O_T_EE[12],msg.O_T_EE[13],msg.O_T_EE[14],euler[0],euler[1],euler[2]]
    # p_pose.pose.orientation.x = initial_quaternion[0]
    # p_pose.pose.orientation.y = initial_quaternion[1]
    # p_pose.pose.orientation.z = initial_quaternion[2]
    # p_pose.pose.orientation.w = initial_quaternion[3]
    # p_pose.pose.position.x = msg.O_T_EE[12]
    # p_pose.pose.position.y = msg.O_T_EE[13]
    # p_pose.pose.position.z = msg.O_T_EE[14]
def text_to_speech(text):
        output_file="output.mp3"
        
        text =text.replace("_", " ")
        tts = gTTS(text=text, lang='en' ,slow=False)
        tts.save(output_file)
        playsound(output_file)    
def gesture_control() :
    
    global gesture_p,mode,scale,p_position,gesture,position,radius,p_pose,repeat
      
    roll,pitch,yaw=euler_from_quaternion(marker_pose.pose.orientation.x,marker_pose.pose.orientation.y,marker_pose.pose.orientation.z,marker_pose.pose.orientation.w) 
    # roll,pitch,yaw=(euler[0],euler[1],euler[1])
   
    # radius=0.305
    
    
     
    if gesture == "slow"  :
        scale=1
        
    elif gesture=="fast" :
        scale=2
    step=0.002*scale
    r_step= 0.2*scale
    r=radius
    # if radius !=0:
    #     r=radius+0.08
    # r=0.3


    circle_step=2*r*(math.sin(math.radians(r_step/2)))
    # circle_step=((2*math.pi*r)*r_step/360)
    position="left"
    # position="right"
    translation_vector = np.array([0, 0, 0])
    rotation_vector = np.array([0, 0, 0])
    # if position =="none":
    #     position="left"
   
    if gesture!=gesture_p  and gesture != "None" :
           
            repeat=0
    else:   
        repeat+=1     
    
    # if   repeat==6 and gesture != "None":
    #     text_to_speech(gesture)
    # print("change = ",repeat)
    if gesture=="ok" and  repeat==6:
        print("grasp")
        grasp_client()
    if gesture != "stop" and gesture != "None" and position != "none"and repeat>1:
         
       
        pose=Pose()
        pose.position.x= translation_vector[0]
        pose.position.y= translation_vector[1]
        pose.position.z= translation_vector[2]
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        t_pose1 = transform_pose(pose, "panda_hand_tcp", "panda_link0")
        print("gesture = {}, radius = {}, position = {}".format(gesture,r,position))
        # for right position        
        if position=="right":
            if gesture=="push" :
                translation_vector = np.array([0, -step, 0])
            elif gesture=="pull" :
                translation_vector = np.array([0, step, 0])     
            elif gesture=="rotate_down" :
                # translation_vector=np.array(calculate_coordinates(0,circle_step,radius))
                translation_vector=np.array([0,0,-circle_step])
                rotation_vector = np.array([0, r_step, 0])     
            elif gesture=="rotate_up" :
                translation_vector=np.array([0,0,circle_step])
                # translation_vector=np.array(calculate_coordinates(0,-circle_step,radius))
                rotation_vector = np.array([0, -r_step, 0])
            elif gesture=="up" :
                translation_vector = np.array([0, 0, -step])
            elif gesture=="down" :
                translation_vector = np.array([0, 0, +step])
            elif gesture=="rotate_counterclockwise" :
                rotation_vector = np.array([0, 0, +r_step])
                translation_vector=np.array([0,circle_step,0])   
                # translation_vector=np.array(calculate_coordinates(-circle_step,0,radius))                                
            elif gesture=="rotate_clockwise" :
                rotation_vector = np.array([0, 0, -r_step])
                translation_vector=np.array([0,-circle_step,0])  
            elif gesture=="left" :
                translation_vector = np.array([-step, 0 ,0])
            elif gesture=="right" :
                translation_vector = np.array([step, 0 ,0])       

                # translation_vector=np.array(calculate_coordinates(circle_step,0,radius)) 
            # elif gesture=="stop" :
            #     translation_vector = np.array([0, 0, 0])    
        # for left position        
        if position=="left":
            if gesture=="pull" :
                translation_vector = np.array([0, -step, 0])
            elif gesture=="push" :
                translation_vector = np.array([0, step, 0])     
            elif gesture=="rotate_up" :
                translation_vector=np.array([0,0,-circle_step])
                # translation_vector=np.array(calculate_coordinates(0,circle_step,radius))
                rotation_vector = np.array([0, r_step, 0])     
            elif gesture=="rotate_down" :
                translation_vector=np.array([0,0,circle_step])
                # translation_vector=np.array(calculate_coordinates(0,-circle_step,radius))
                rotation_vector = np.array([0, -r_step, 0])
            elif gesture=="up" :
                translation_vector = np.array([0, 0, -step])
            elif gesture=="down" :
                translation_vector = np.array([0, 0, +step])
            elif gesture=="rotate_counterclockwise" :
                rotation_vector = np.array([0, 0, +r_step])
                translation_vector=np.array([0,circle_step,0])
                # translation_vector=np.array(calculate_coordinates(-circle_step,0,radius))                                
            elif gesture=="rotate_clockwise" :
                rotation_vector = np.array([0, 0, -r_step])
                translation_vector=np.array([0,-circle_step,0]) 
            elif gesture=="left" :
                translation_vector = np.array([step, 0 ,0])
            elif gesture=="right" :
                translation_vector = np.array([-step, 0 ,0])     
                # translation_vector=np.array(calculate_coordinates(circle_step,0,radius)) 
            # elif gesture=="stop" :
            #     translation_vector = np.array([0, 0, 0])
        # for centre position        
        if position=="center":
            radius=0
            if gesture=="push" :
                translation_vector = np.array([-step,0,0])
            elif gesture=="pull" :
                translation_vector = np.array([step, 0,0])     
            elif gesture=="rotate_up" :
                rotation_vector = np.array([r_step,0, 0])     
            elif gesture=="rotate_down" :
                rotation_vector = np.array([-r_step,0,  0])
            elif gesture=="up" :
                translation_vector = np.array([0, 0, -step])
            elif gesture=="down" :
                translation_vector = np.array([0, 0, +step])
            elif gesture=="rotate_counterclockwise" :
                rotation_vector = np.array([0, 0, +r_step])
                translation_vector=np.array([0,circle_step,0])                               
            elif gesture=="rotate_clockwise" :
                rotation_vector = np.array([0, 0, -r_step])
                translation_vector=np.array([0,-circle_step,0])
            elif gesture=="left" :
                translation_vector = np.array([0,step ,0])
            elif gesture=="right" :
                translation_vector = np.array([0,-step ,0])     
            # elif gesture=="stop" :
            #     translation_vector = np.array([0, 0, 0])          
        # print("translational vector = {} , rotational vector = {} , gesture =  {}, position ={}, radius = {} ".format(translation_vector,rotation_vector,gesture,position,radius))                                                                   
            
        
        my_pose=Pose()
        my_pose.position.x= translation_vector[0]
        my_pose.position.y= translation_vector[1]
        my_pose.position.z= translation_vector[2]
        my_pose.orientation.x = 0
        my_pose.orientation.y = 0
        my_pose.orientation.z = 0
        my_pose.orientation.w = 1
        t_pose = transform_pose(my_pose, "panda_hand_tcp", "panda_link0")
        # t_pose=marker_pose.pose
        # print("gesture=",gesture,"t_pose=",t_pose.position.x,t_pose.position.y,t_pose.position.z)
        # print("translation = ",translation_vector)
        roll+=math.radians(rotation_vector[0])
        pitch+=math.radians(rotation_vector[1])
        yaw+=math.radians(rotation_vector[2])
        # roll+=rotation_vector[0]
        # pitch+=rotation_vector[1]
        # yaw+=rotation_vector[2]
        rot=transform.quaternion_from_euler(roll,pitch,yaw,'sxyz') 
        rot = rot / np.linalg.norm(rot)  
        error_x=t_pose.position.x - t_pose1.position.x
        error_y=t_pose.position.y - t_pose1.position.y
        error_z=t_pose.position.z - t_pose1.position.z
        marker_pose.pose.position.x += error_x
        marker_pose.pose.position.y += error_y
        marker_pose.pose.position.z += error_z
        marker_pose.pose.orientation.x = rot[0]
        marker_pose.pose.orientation.y = rot[1]
        marker_pose.pose.orientation.z = rot[2]
        marker_pose.pose.orientation.w = rot[3] 
            
        # marker_pose.pose=t_pose  
        marker_pose.pose.position.x = max([min([marker_pose.pose.position.x,
                                                position_limits[0][1]]),
                                                position_limits[0][0]])
        marker_pose.pose.position.y = max([min([marker_pose.pose.position.y,
                                                position_limits[0][1]]),
                                                position_limits[0][0]])
        marker_pose.pose.position.z = max([min([marker_pose.pose.position.z,
                                                position_limits[0][1]]),
                                                position_limits[0][0]])
        # pose_base=transform_pose(marker_pose.pose, "panda_hand_tcp", "panda_link0")
        # marker_pose.pose = pose_base
        # int_marker.pose = marker_pose.pose
        
        p_position =position
        pose_pub.publish(marker_pose)
        
        # server.applyChanges()
    radius=0
    gesture_p=gesture
    
        
    
def position_callback(msg):
    global position,radius,p_position,R_pose,radius_pose,p_repeat
    camera_2=msg
    position=camera_2.header.frame_id
    # if  camera_2.header.frame_id !="none":   
        # position=camera_2.header.frame_id
    # else:
    #     position=p_position    
    radius=camera_2.point.x
    # print(camera_2)
    print("position = {} radiaus= {}".format(position,radius))
    if position!=p_position and position!="none" and position!='':
        #    text_to_speech(f"{'command position '}{position}") 
        #    print(position)
        p_repeat=0
    else:   
        p_repeat+=1   
    if  p_repeat==3 and position != "None":
        text_to_speech(f"{'command position '}{position}") 
    
    
    # r=0
    # if radius!=0:
    #     r=radius+0.04     
    p_position =position
    # if radius !=0:
    # radius_pose=Pose()

    radius_pose.position.x=camera_2.point.x
    radius_pose.position.y=camera_2.point.y
    radius_pose.position.z=camera_2.point.z
    radius_pose.orientation.w=1
    # R_pose=PoseStamped()
   
   
  
    # print(R_pose)
    # radius_pub.publish(R_pose)
    # radius_publisher()
    
def radius_publisher():
    global R_pose,radius_pose
    # print("radius_pose ",radius_pose)
    R_pose.pose = transform_pose(radius_pose, "zedm_left_camera_frame", "panda_link0")
    radius_pub.publish(R_pose)
    radius_pose.position.x=0
    radius_pose.position.y=0
    radius_pose.position.z=0
    radius_pose.orientation.w=1
  

def processFeedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        marker_pose.pose.position.x = max([min([feedback.pose.position.x,
                                          position_limits[0][1]]),
                                          position_limits[0][0]])
        marker_pose.pose.position.y = max([min([feedback.pose.position.y,
                                          position_limits[1][1]]),
                                          position_limits[1][0]])
        marker_pose.pose.position.z = max([min([feedback.pose.position.z,
                                          position_limits[2][1]]),
                                          position_limits[2][0]])
        marker_pose.pose.orientation = feedback.pose.orientation
    pose_pub.publish(marker_pose)  
    server.applyChanges()


if __name__ == "__main__":
    # global R_pose
    rospy.init_node("equilibrium_pose_node")
    
    pose_sub = rospy.Subscriber("/gesture_pose", String, gesture_callback,queue_size=10)
    gripper_sub = rospy.Subscriber("/franka_gripper/joint_states",
                                JointState, gripper_state_callback,queue_size=10)
    position_sub = rospy.Subscriber("/position_publisher", PointStamped, position_callback,queue_size=10)
    state_sub_sub = rospy.Subscriber("franka_state_controller/franka_states",
                                FrankaState, franka_state_callback,queue_size=10)
    listener = tf.TransformListener()
    
    link_name = rospy.get_param("~link_name")
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    # Get initial pose for the interactive marker
    while not initial_pose_found:
       rospy.sleep(1)
    #state_sub.unregister()
    R_pose.header.frame_id = "panda_link0"
    R_pose.header.stamp = rospy.Time(0)
    pose_pub = rospy.Publisher("equilibrium_pose", PoseStamped, queue_size=10)
    radius_pub = rospy.Publisher("Radius", PoseStamped, queue_size=10)
    server = InteractiveMarkerServer("equilibrium_pose_marker")
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = link_name
    int_marker.scale = 0.3
    int_marker.name = "equilibrium_pose"
    int_marker.description = ("Equilibrium Pose\nBE CAREFUL! "
                            "If you move the \nequilibrium "
                            "pose the robot will follow it\n"
                            "so be aware of potential collisions")
    int_marker.pose = marker_pose.pose
    # run pose publisher
    rospy.Timer(rospy.Duration(0.01),
                lambda msg: publisherCallback(msg, link_name))
    rospy.Timer(rospy.Duration(0.1),
                lambda msg: gesture_control())
    rospy.Timer(rospy.Duration(0.1),
                lambda msg: radius_publisher())
    # rospy.Timer(rospy.Duration(0.02),
    #              franka_state_callback(FrankaState))
    #insert a box
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.always_visible=True
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.always_visible=True
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.always_visible=True
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.always_visible=True
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.always_visible=True
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.always_visible=True
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    server.insert(int_marker, processFeedback)
    

    server.applyChanges()

    rospy.spin()
