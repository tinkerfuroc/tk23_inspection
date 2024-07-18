import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
import tf2_geometry_msgs
from tf2_ros import TransformException

from tinker_vision_msgs.msg import Object, Objects, Face, FaceResult
from tinker_vision_msgs.srv import FaceRegister
from tinker_arm_msgs.srv import URControlService, Robotiq
from audio_common_msgs.msg import SoundRequest
from tts_interfaces.srv import TTSRequest

import numpy as np
import time

class Person:
    def __init__(self, id, name, drink):
        self.id = None
        self.name = None
        self.drink = None

    def desp_gen(self):
        return f"This is {self.name}, {self.name}'s favourite drink is {self.drink}"
    
    def ensure_gen(self):
        return f"Hi! Your name is {self.name} and your favourite drink is {self.drink}, right?"


def dist(x: Point, y: Point):
    return np.sqrt((x.x - y.x) ** 2 + (x.y - y.y) ** 2 + (x.z - y.z) ** 2)

class Receptionist(Node):

    def __init__(self):
        super().__init__('receptionist')
        self.stage = "init_task"

        self.cnt = 0  # record for the num of people
        self.guestnum = 0
        # self.host_info = ["Robin", "Coffee"]
        self.faces: FaceResult = FaceResult()
        self.face: Face = Face()

        # Person
        self.host = Person()
        self.guest1 = Person()
        self.guest2 = Person()

        self.face_register_client = self.create_client(FaceRegister, 'vision/face/register')
        self.ur_client = self.create_client(URControlService, 'tinker_arm_control_service')
        self.gripper_client = self.create_client(Robotiq, 'robotiq_service')
        self.tts_client = self.create_client(TTSRequest, 'TTSRequest')
        # self.odom_sub = self.create_subscription(Odometry, 'odometry/filtered', self.odom_callback, 10)
        # self.current_pose = None

        # self.navigation_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # self.door_position = Pose()
        # self.sofa_position = Pose()

    def speak(self, content):
        req = TTSRequest.Request()
        req.target_string = content
        self.tts_client.call_async(req)
        return True        
    
    # def odom_callback(self, odometry):
    #     # type() = Pose
    #     try:
    #         transform = self.tf_buffer.lookup_transform(
    #             target_frame='map',
    #             source_frame='odom',
    #             time=rclpy.time.Time()
    #         )
    #     except TransformException as ex:
    #         self.get_logger().warn(f'Could not transform /odom to /map: {ex}')
    #         return
        
    #     # odometry = Odometry
    #     # odometry.pose = PoseWithCovariance
    #     # odometry.pose.pose = Pose
    #     self.current_pose = tf2_geometry_msgs.do_transform_pose(odometry.pose.pose, transform)
    #     # self.get_logger().warn('Update on current_robot_position = %.3f, %.3f, %.3f' % (self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z))
    
    # def navigation_cmd(self, pose: Pose):
    #     msg = PoseStamped()
    #     msg.pose = pose
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.header.frame_id = 'map'
        
    #     if not self.debug:
    #         self.navigation_pub.publish(msg)
    

    def task(self):
        if self.stage == "init_task":
            success = self.speak("Hello! My name is Tinker. I'm going to welcome the guests!")
            time.sleep(8)
            if success:
                self.stage = "register_host"
        
        elif self.stage == "register_host":
            print("Begin register...")
            # self.face_register_client.wait_for_server()   #
            register_req = FaceRegister.Request()
            register_req.info = ['register']
            future = self.face_register_client.call_async(register_req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            # res: FaceRegisterResponse = self.vision_face_register(self.host_info)
            if res.success:
                print("Successfully registered!")
                self.speak("Hello, Alex! I successfully register your face!")
                time.sleep(10)
                self.guestnum = 1
                self.stage = "go_to_door"
            else:
                if res.fail_info == "Cannot detect any faces.":
                    self.speak("I cannot see host. Could you please stand in front of me?")
                elif res.fail_info == "Detect more than one face.":
                    self.speak("I detect more than one face. Please leave my sight if you are not the host.")

        elif self.stage == "go_to_door":
            # TODO: ADD NAVIGATION
            print("Going to the door.")
            time.sleep(17)
            if self.guestnum == 1:
                self.speak("I am waiting for the first guest.")
                self.stage = "wait for guest1 info"
            elif self.guestnum == 2:
                self.speak("I am waiting for the second guest.")
                self.stage = "wait for guest2 info"

        elif self.stage == "wait for guest1 info":
            success = self.speak("Hello, I am Tinker. Could you please tell me your name and favourite drink?")
            if success:
                self.stage = "register_guest1"   

        elif self.stage == "register_guest1":   # for at least 2 people
            # guest_info is from audio message
            time.sleep(16)
            register_req = FaceRegister.Request()
            register_req.info = ['register']
            future = self.face_register_client.call_async(register_req)   
            rclpy.spin_until_future_complete(self, future)
            res = future.result()

            # res: FaceRegister = self.vision_face_register(self.guest_info)
            if res.success:
                self.cnt += 1   
                self.stage = "ensure_guest1"
            else:
                if res.fail_info == "Cannot detect any faces.":
                    self.speak("I cannot see you. Could you please stand in front of me?")
                elif res.fail_info == "Detect more than one face.":
                    self.speak("I detect more than one face. Please leave my sight if you are not the guest.")

        elif self.stage == "ensure_guest1":
            self.speak(self.guest1.ensure_gen())
            time.sleep(9)
            success = self.speak("Please follow me.")
            self.stage = "go_to_sofa"
            # if success:
            #     if self.cnt == 2:
            #         self.stage = "go_to_sofa"
            #     elif self.cnt == 1: 
            #         self.stage = "wait for guest info"
            #     elif self.cnt == 0:
            #         print("No registered guest.")
            #     else:
            #         print("Error: cnt > 2")

        elif self.stage == "go_to_sofa":
            print("Going to sofa.")
            # TODO: ADD NAVIGATION
            time.sleep(40)
            self.speak(f"We arrive the living room. Please stand on my left, {self.host.name}!")
            time.sleep(8)
            self.stage = "introduce_guest1"
            ## to do : add nav (to sofa)
            # success = True
            # if success:
            #     self.stage = "recognize_guest"
                
        # elif self.stage == "recognize_guest":
        #     print("Recognizing guest...")
        #     time.sleep(5)
        #     success = True
        #     if success:
        #         self.stage = "introduce"

        elif self.stage == "introduce_guest1":
            # Alex is the host's name, Mike is the guest's name
            # TODO: ADD FEATURE
            success = self.speak("Hello, Alex! This is Mike, and his favourite drink is milk. He is a around 20 years old. He is wearing brown clothes today. His hair is black.")
            time.sleep(25)
            success = self.speak("Hello, Mike! This is the host Alex, and his favourite drink is cola. He is a around 25 years old. He is wearing black clothes today. His hair is black.")
            self.stage = "point_seat1"

        elif self.stage == "point_seat1":
            self.speak("Please sit in your left on the sofa!")
            time.sleep(5)
            # ADD UR action

            # ur_req = URControlService.Request()
            # ur_req.target_pose = Pose(position=Point(x=0.5,y=-0.4,z=1.2),orientation=Quaternion(x=0.,y=0.707,z=0.,w=0.707))
            # arm_future = self.ur_client.call_async(ur_req)
            # rclpy.spin_until_future_complete(self, arm_future)
            # res = arm_future.result()
            # time.sleep(5)
            # ur_req.target_pose = Pose(position=Point(x=0.4,y=-0.12,z=1.1),orientation=Quaternion(x=0.,y=0.,z=0.,w=1.0))
            # arm_future = self.ur_client.call_async(ur_req)
            # rclpy.spin_until_future_complete(self, arm_future)
            # res = arm_future.result()
            self.stage = "register_guest2"

        elif self.stage == "register_guest2":
            self.speak("Now I am going to the door to welcome next guest.")
            # TODO: ADD NAVIGATION
            time.sleep(20)
            self.speak("Hello, I am Tinker. Could you please tell me your name and favourite drink?")
            time.sleep(17)
            register_req = FaceRegister.Request()
            register_req.info = ['register']
            future = self.face_register_client.call_async(register_req)   
            rclpy.spin_until_future_complete(self, future)
            res = future.result()

            # res: FaceRegister = self.vision_face_register(self.guest_info)
            if res.success:
                self.cnt += 1   
                self.stage = "ensure_guest1"
            else:
                if res.fail_info == "Cannot detect any faces.":
                    self.speak("I cannot see you. Could you please stand in front of me?")
                elif res.fail_info == "Detect more than one face.":
                    self.speak("I detect more than one face. Please leave my sight if you are not the guest.")

            self.speak("Hi! Your name is Mary and your favourite drink is orange juice, right?")
            time.sleep(9)
            success = self.speak("Please follow me.")
            time.sleep(40)

            self.stage = "introduce_guest2"

        elif self.stage =="introduce_guest2":
            self.speak("We arrive the living room. Please stand on my left, Mike!")
            time.sleep(7)
            success = self.speak("Hello, Alex! This is Mary, and her favourite drink is orange juice. She is a around 26 years old. She is wearing green clothes today. Her hair is black.")
            time.sleep(17)
            success = self.speak("Hello, Mike! This is Mary, and her favourite drink is orange juice. She is a around 26 years old. She is wearing green clothes today. Her hair is black.")
            time.sleep(18)
            success = self.speak("Hello, Mary! This is the host Alex, and his favourite drink is cola. He is a around 25 years old. He is wearing black clothes today. His hair is black.")
            success = self.speak("This is the first guest Mike, and his favourite drink is milk. He is a around 20 years old. He is wearing brown clothes today. His hair is black.")
            
            self.stage = "point_seat2"

        elif self.stage == "point_seat2":
            self.speak("Please sit in the middle of sofa!")
            ur_req = URControlService.Request()
            # ur_req.target_pose = Pose(position=Point(x=0.5,y=0.2,z=1.2),orientation=Quaternion(x=0.,y=0.707,z=0.,w=0.707))
            # arm_future = self.ur_client.call_async(ur_req)
            # rclpy.spin_until_future_complete(self, arm_future)
            # res = arm_future.result()
            # time.sleep(5)
            # ur_req.target_pose = Pose(position=Point(x=0.4,y=-0.12,z=1.1),orientation=Quaternion(x=0.,y=0.,z=0.,w=1.0))
            # arm_future = self.ur_client.call_async(ur_req)
            # rclpy.spin_until_future_complete(self, arm_future)
            # res = arm_future.result()
            self.stage = "finished"

        
        return(self.stage)
            



def main(args=None):
    rclpy.init(args=args)

    receptionist_node = Receptionist()

    while receptionist_node.task() != "finished":
        continue

    receptionist_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()