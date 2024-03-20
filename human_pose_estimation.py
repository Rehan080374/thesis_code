#!/usr/bin/env python3
import sys
import pyzed.sl as sl
import os
import cv2
import math
import numpy as np
from detection import HandDetector
import rospy
import signal
from geometry_msgs.msg import PointStamped
from gtts import gTTS
from playsound import playsound
class PositionTracker:
    def __init__(self):
        self.object_length = 0.8
        self.work_space = 2
        # self.p_position="none"
        # self.output_file = "output_position.mp3"
        self.point = PointStamped()
        self.hand_detector = HandDetector(maxHands=1, detectionCon=0.5)
        self.init_ros()
        self.init_zed_camera()
        self.distance=[0,0,0,0]

    def init_ros(self):
        rospy.init_node('position_node')
        self.pub = rospy.Publisher("/position_publisher", PointStamped, queue_size=10)
        signal.signal(signal.SIGINT, self.sigint_handler)
    def init_zed_camera(self):
        init_params = sl.InitParameters()
        # Configure the ZED camera parameters
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.coordinate_units = sl.UNIT.METER
        init_params.camera_fps = 30
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_params.depth_minimum_distance = 0.1

        self.zed = sl.Camera()
        
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            rospy.logerr("Failed to open ZED camera.")
            sys.exit(1)

        # Enable object detection and tracking
        obj_param = sl.ObjectDetectionParameters()
        obj_param.enable_tracking = True
        if obj_param.enable_tracking:
            self.zed.enable_positional_tracking()
        self.zed.enable_object_detection(obj_param)    
        camera_info = self.zed.get_camera_information()
    
        self.width=camera_info.camera_resolution.width
        self.height=camera_info.camera_resolution.height
        # Configure object detection runtime parameters
        self.obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        self.obj_runtime_param.detection_confidence_threshold = 60
        self.obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]    # Only detect Persons
    def sigint_handler(self, signum, frame):
        rospy.loginfo("Shutting down gracefully...")
        self.zed.close()
        rospy.signal_shutdown("SIGINT received")
    # def text_to_speech(self,text):
        
    #     mod_text = f"{'command position '}{text}"
    #     # mod_text ="".join([s, text])
    #     tts = gTTS(text=mod_text, lang='en' ,slow=False)
    #     tts.save(self.output_file)
    #     playsound(self.output_file)
    def check_depth(self, depth, point):
        
        dist=0
        dist_array=[]
        x,y=point
        neighbors=[[x,y],[x-1,y-1],[x-1,y],[x-1,y+1],[x,y-1],[x,y+1],[x+1,y+1],[x+1,y],[x+1,y-1]]
        # for j in range(0,len(neighbors)):
        #     if neighbors[j][0]>=0 and neighbors[j][1]>=0:
        #         e,d = depth.get_value(neighbors[j][0],neighbors[j][1])
        #         dist_array.append(d)
                # if not np.isnan(dist) and not np.isinf(dist) and dist>=0  :          
                #     break 
        # if not np.isnan(dist) and not np.isinf(dist) and dist>=0  :          
        #         dist=dist
        # else:
        #             dist=0
        # dist=min(dist_array) 
        if point[0]>=0 and point[1]>=0:
            e,dist = depth.get_value(x,y)
        if abs(dist-self.distance)>1 or  np.isnan(dist):
            dist=self.distance
        self.distance=dist               
        return dist   
    def check_point_cloud(self, point_cloud, point):
        
        min_dist=0
        a = []
        b = []
        c = []

        x,y=point
        neighbors=[[x,y],[x-1,y-1],[x-1,y],[x-1,y+1],[x,y-1],[x,y+1],[x+1,y+1],[x+1,y],[x+1,y-1]]
        for j in range(0,len(neighbors)):
            if neighbors[j][0]>=0 and neighbors[j][1]>=0:
                e,p= point_cloud.get_value(neighbors[j][0],neighbors[j][1])
                a.append(p[0])
                b.append(p[1])
                c.append(p[2])
                # if not np.isnan(dist) and not np.isinf(dist) and dist>=0  :          
                #     break 
        if len(a)>1 and len(b)>1 and len(c)>1 :        
            x_min=min(a)
            y_min=min(b)
            z_min=min(c)
        else:
            x_min=0
            y_min=0
            z_min=0
        min_dist = math.sqrt(x_min * x_min + y_min * y_min + z_min * z_min)

        # if not np.isnan(dist) and not np.isinf(dist) and dist>=0  :          
        #         dist=dist
        # else:
        #             dist=0
        # dist=min(dist_array) 
      
        if abs(min_dist-self.distance[0])>1 or  np.isnan(min_dist):
            min_dist,x_min,y_min,z_min=self.distance
        self.distance=[min_dist,x_min,y_min,z_min]       
        dist=[min_dist,x_min,y_min,z_min]     
        return dist 
    def run(self):
        depth = sl.Mat()
        image = sl.Mat()
        objects = sl.Objects()
        point_cloud=sl.Mat()

        tracking_active = False  # Flag to indicate if tracking is active
        tracked_person_id = None  # ID of the currently tracked person
        tracking_range = 0.5  # Distance threshold to start tracking (in meters)
        i=0
        while not rospy.is_shutdown():
            i+=1
            if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(image, sl.VIEW.LEFT)
                img = image.get_data()

                self.zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
                self.zed.retrieve_objects(objects, self.obj_runtime_param)

                if not tracking_active:
                    # Look for the first person in view and range to start tracking
                    for object in objects.object_list:
                        object_label=object.label
                        object_position=object.position
                        object_id=object.id
                        
                        if  object_position[0] <= tracking_range:
                            tracking_active = True
                            tracked_person_id = object.id
                            uniqe_id= sl.generate_unique_id() 
                            print(uniqe_id)	
                            rospy.loginfo("Tracking person with ID: %d", tracked_person_id)
                            break

                if tracking_active and object.tracking_state:
                    tracked_person_found = False
    
                    for object in objects.object_list:
                       
                        if object.id == tracked_person_id :
                            tracked_person_found = True
                            bbox= object.bounding_box_2d
                            bbox = np.asarray(bbox, dtype = 'int')
                            # print("bbox= ",bbox)
                            position="none"
                            object_position=object.position
                            bbox= object.bounding_box_2d
                            left_side_distance=bbox[0][0]
                            right_side_distance=self.width-bbox[1][0]
                            bbox = np.asarray(bbox, dtype = 'int')
                            cv2.rectangle(img, (int(bbox[0][0]),int(bbox[0][1])),(int(bbox[2][0]),int(bbox[2][1])),
                                                        (255, 0, 255), 5) 
                            # print("object position {} object id {} object label  {} ".format(object_position,object.id,object.label))
                            # print("left: ",left_side_distance," right : ",right_side_distance)
                            if object_position[0]<=self.work_space :
                                if  left_side_distance>=50 and right_side_distance>=50 and object_position[0]>self.object_length:
                                    position="center"
                                elif object_position[1] >0 and left_side_distance<right_side_distance:
                                    position="left"
                                elif object_position[1] <0 and right_side_distance<left_side_distance:
                                    position="right"
                                else:
                                    position="none"
                                self.point.header.frame_id=position    
                                
                                # print("bbox= ",bbox)
                                object_id=object.id
                                id=f"{'object_id: '}{object.id}"
                                cv2.putText(img, id, ((bbox[0][0]) + 100, bbox[0][1]  +100), cv2.FONT_HERSHEY_PLAIN,
                                                    2, (255, 0, 255), 2) 
                                cv2.putText(img, f"{'command position: '}{position}", ((bbox[0][0]) + 100, bbox[0][1]  +120), cv2.FONT_HERSHEY_PLAIN,
                                                    2, (255, 0, 255), 2)
                                cv2.imwrite(f'{"/home/panda/model_data/test/image_test"}_{i}.jpg',img)

                                hands, img = self.hand_detector.findHands(img, blk=False) 
                                for hand in hands:
                                    lm=hand["lmList"]
                                    # print(lm[0][0]*width,lm[0][1]*height)
                                    # center=hand["center"]
                                    if (hand["type"]=="Left" and position=="right") or (hand["type"]=="Right" and position=="left"):
                                        center=[lm[9][0]*self.width,lm[9][1]*self.height]
                                        # print(center)
                                        # distance =round(self.check_depth(point_cloud,center),3)
                                        distance,x,y,z= self.check_point_cloud(point_cloud,center)
                                        # x = round(point3D[0],4)
                                        # y = round(point3D[1],4)
                                        # z = round(point3D[2],4)  
                                        # print("x={} , y={} , z={} and distnce={}".format(x,y,z,distance)) 
                                        # print("3d point = ",point3D)
                                        if distance >=self.object_length  :
                                            distance =0
                                            x,y,z=[0,0,0]
                                        self.point.point.x=x
                                        self.point.point.y=y 
                                        self.point.point.z=z 
                                        
                                        # r=('radius' ,distance)
                                        # print(r)
                                        cv2.putText(img, f"{'Radius: '}{distance}", ((bbox[0][0]) +100, bbox[0][1] +150), cv2.FONT_HERSHEY_PLAIN,
                                                            2, (255, 0, 255), 2)   
                                        
                                        
                                        
                                    else:
                                        self.point.point.x=0
                                        self.point.point.y=0
                                        self.point.point.z=0       
                            break
                    self.pub.publish(self.point)
                    if not tracked_person_found:
                        rospy.loginfo("Tracked person lost or moved out of view.")
                        tracking_active = False
                        tracked_person_id = None

                cv2.imshow('Position tracking', img)
                sys.stdout.flush()
                
                if cv2.waitKey(30) >= 0 or rospy.is_shutdown():
                    print("Shutting down")
                    cv2.destroyAllWindows()
                    break

        rospy.loginfo("Node is shutting down.")

        self.zed.disable_object_detection()
        self.zed.disable_positional_tracking()
        self.zed.close()
if __name__ == "__main__":
    position_tracker = PositionTracker()
    position_tracker.run()
