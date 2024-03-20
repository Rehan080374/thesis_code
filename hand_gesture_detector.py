#!/usr/bin/env python3
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
import cv2
import numpy as np
import math
import mediapipe as mp
import pickle
import rospy
from std_msgs.msg import String
import signal
from detection import HandDetector
from keras.models import load_model
from gtts import gTTS
from playsound import playsound
import tensorflow as tf
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        tf.config.experimental.set_memory_growth(gpus[0], True)
    except RuntimeError as e:
        print(e)
class GestureDetector:
    def __init__(self):
        self.handDetector = HandDetector()
        self.init_ros()
        self.init_camera()
        # self.output_file = "output.mp3"
        # self.p_label="none"
        # self._init_hand_detector()
    def init_ros(self):
        rospy.init_node("gesture_node")
        self.gesture_pub = rospy.Publisher("gesture_pose", String, queue_size=10, tcp_nodelay=True)
        signal.signal(signal.SIGINT, self.sigint_handler)
    def init_camera(self):
        self.capture = cv2.VideoCapture(0, cv2.CAP_V4L2)
        # self.capture = cv2.VideoCapture(0)
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.width, self.height = 1600,900
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv2.CAP_PROP_FPS, 60) 
       
        self.fps = self.capture.get(cv2.CAP_PROP_FPS)
        self.resolution=(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH),self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.width, self.height = self.resolution
        print("Current FPS =", self.fps)
        print("Current resolutuion =", self.resolution)
    def _init_hand_detector(self, mode=False, maxHands=2, detectionCon=0.5, minTrackCon=0.5):
        """
        :param mode: In static mode, detection is done on each image: slower
        :param maxHands: Maximum number of hands to detect
        :param detectionCon: Minimum Detection Confidence Threshold
        :param minTrackCon: Minimum Tracking Confidence Threshold
        """
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.minTrackCon = minTrackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=self.mode, max_num_hands=self.maxHands,
                                        min_detection_confidence=self.detectionCon,
                                        min_tracking_confidence=self.minTrackCon)
        self.mpDraw = mp.solutions.drawing_utils
  
        self.lmList = []   
    def sigint_handler(self, signum, frame):
        rospy.loginfo("Shutting down gracefully...")
        rospy.signal_shutdown("SIGINT received")
    # def text_to_speech(self,text):
    #     mod_text = text.replace("_", " ")
    #     tts = gTTS(text=mod_text, lang='en' ,slow=False)
    #     tts.save(self.output_file)
    #     playsound(self.output_file)
    def load_model_and_data(self):
        folder = "/home/panda/model_data/model9/"
        model_path = os.path.join(folder, "keras_model.h5")
        label_path = os.path.join(folder, "labels.txt")
        scaler_file = os.path.join(folder, "scaler.pkl")
        normalizer_file = os.path.join(folder, "normalizer.pkl")

        self.model = load_model(model_path)
        with open(label_path, 'r') as f:
            self.labels = f.read().splitlines()

        with open(scaler_file, 'rb') as scaler_file:
            self.scaler = pickle.load(scaler_file)

        with open(normalizer_file, 'rb') as normalizer_file:
            self.normalizer = pickle.load(normalizer_file)

    def process_frame(self, img):
        hands, img = self.handDetector.findHands(img, blk=False)

        if len(hands) >= 2:
            combined_array= self.process_detected_hands(hands)
            if len(combined_array) == 210:
                label = self.predict_gesture_label(combined_array)
                self.publish_gesture_label(label, img)

        cv2.imshow("Frame", img)

    def process_detected_hands(self, hands):
        left_hand_landmarks = []
        left_hand_distances = []
        right_hand_landmarks = []
        right_hand_distances = []

    
        left_hand = None
        right_hand = None

        # Process the detected hands
        for hand in hands:
            hand_type = hand["type"]
            if hand_type == "Left" and left_hand is None:
                left_hand = hand
                left_hand_landmarks.extend(left_hand["lmList"])
                # print("left hand found")
            elif hand_type == "Right" and right_hand is None:
                right_hand = hand
                right_hand_landmarks.extend(right_hand["lmList"])
                # print("right hand found")

        # Collect landmarks and calculate distances for left hand
        if left_hand is not None and right_hand is not None:
            for left_landmark,right_landmarks in zip(left_hand_landmarks,right_hand_landmarks):
                try:
                    l_length = self.calculate_distance(left_hand_landmarks[0], left_landmark)
                    left_hand_distances.append(l_length)
                    
                    length_with_right = self.calculate_distance(right_hand_landmarks[0], left_landmark)
                    left_hand_distances.append(length_with_right)

                    r_length = self.calculate_distance(right_hand_landmarks[0], right_landmarks)
                    right_hand_distances.append(r_length)

                    length_with_left = self.calculate_distance(left_hand_landmarks[0], right_landmarks)
                    right_hand_distances.append(length_with_left)
                except Exception as e:
                    print(e)

           

        # Convert the collected landmarks and distances to flattened NumPy arrays
        left_hand_landmarks = np.array(left_hand_landmarks).flatten()
        left_hand_distances = np.array(left_hand_distances)
        right_hand_landmarks = np.array(right_hand_landmarks).flatten()
        right_hand_distances = np.array(right_hand_distances)

        # Concatenate the arrays in the specified order
        combined_array = np.concatenate((left_hand_landmarks, left_hand_distances,
                                        right_hand_landmarks, right_hand_distances))

        return combined_array

    def calculate_distance(self,landmark1, landmark2):
            """
            Calculates the Euclidean distance between two landmarks.
            :param landmark1: First landmark in the format [x1, y1, z1]
            :param landmark2: Second landmark in the format [x2, y2, z2]
            :return: Distance between the landmarks
            """
            x1, y1, z1 = landmark1
            x2, y2, z2 = landmark2

            distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
            return distance
    def predict_gesture_label(self, combined_array):
        input = combined_array.reshape((1, 210)) 
        data_normalized = self.normalizer.transform(self.scaler.transform(input))
        predictions = self.model.predict(data_normalized)
        predicted_label_index = np.argmax(predictions)
        predicted_label = self.labels[predicted_label_index]
        
        confidence_values = predictions[0][predicted_label_index]  # Assuming predictions is a 2D array
        
        confidence_percentiles = confidence_values * 100
        print(f"Label: {predicted_label}, Confidence: {confidence_percentiles}")
        # if confidence_percentiles < 95 and predicted_label!="stop" and predicted_label!="left":
        #     predicted_label="None"

        return predicted_label

    def publish_gesture_label(self, label,img):
        # cv2.putText(blk_img, label, (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 100, 0), 2)
        cv2.putText(img, label, (150, 150), cv2.FONT_HERSHEY_PLAIN, 5, (255, 100, 0), 5)
        self.gesture_pub.publish(label)
        # if label !=  self.p_label and label!="None":
        #     self.text_to_speech(label)
        #self.p_label=label

    def run(self):
        self.load_model_and_data()
            

        while not rospy.is_shutdown():
            success, img = self.capture.read()
            self.process_frame(img)

            if cv2.waitKey(30) >= 0:
                break

        cv2.destroyAllWindows()
        rospy.loginfo("Node is shutting down.")

if __name__ == "__main__":
    gesture_detector = GestureDetector()
    gesture_detector.run()
