from statemachine import StateMachine, State
from statemachine.contrib.diagram import DotGraphMachine

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from threading import Event

from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
import tf2_geometry_msgs
from tf2_ros import TransformException

from tinker_vision_msgs.msg import Object, Objects, Face, FaceResult
from tinker_vision_msgs.srv import FaceRegister
from tinker_arm_msgs.srv import URControlService, Robotiq
from tts_interfaces.srv import TTSRequest

import numpy as np
import time
from math import sin, cos
from receptionist_task import Receptionist_task
class Person:
    def __init__(self, id, name, drink):
        self.id = None
        self.name = None
        self.drink = None

    def desp_gen(self):
        return f"This is {self.name}, {self.name}'s favourite drink is {self.drink}"
    
    def ensure_gen(self):
        return f"Hi! Your name is {self.name} and your favourite drink is {self.drink}, right?"

class Receptionist(Node, StateMachine):

    def __init__(self):
        # Init Node
        super().__init__('receptionist')
        self.faces: FaceResult = FaceResult()
        self.face: Face = Face()
        self.face_register_client = self.create_client(FaceRegister, 'vision/face/register')
        self.ur_client = self.create_client(URControlService, 'tinker_arm_control_service')
        self.gripper_client = self.create_client(Robotiq, 'robotiq_service')
        self.tts_client = self.create_client(TTSRequest, 'TTSRequest')
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        
        # Init State machine
        self.host = Person()
        self.guest1 = Person()
        self.guest2 = Person()
        persons = [self.host, self.guest1, self.guest2]
        locations = ['door', 'sofa']
        # Define action attribute
        person_cnt = 0
        location_lable = 0
        active_person = persons[person_cnt]
        active_location = locations[location_lable]
        # Define states
        init_task = State('Init Task', initial=True)
        register = State('Register')
        move = State('Move')
        introduce = State('Introduce')
        gesture = State('Gesture')
        finished = State('Finished',final=True)
        self.done_event = Event()

        # Define transitions
        start_register = init_task.to(register)
        start_move = register.to(move)
        start_introduce = move.to(introduce)
        start_gesture = introduce.to(gesture)
        return_to_register = move.to(register)
        return_to_move = gesture.to(move)
        finish_task = gesture.to(finished)

    ##### Function for Node ##########
    def speak(self, content):
        req = TTSRequest.Request()
        req.target_string = content
        self.tts_client.call_async(req)
        return True   

    def send_nav_goal(self, x, y, theta):
        self.done_event.clear()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = cos(theta / 2.0)

        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)  

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Nav Goal rejected :(')
            self.done_event.set()
            return

        self.node.get_logger().info('Nav Goal accepted :)')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('Navigation succeeded!')
        else:
            self.node.get_logger().info('Navigation failed...')
        self.done_event.set()   

    ##### Function for State Machine ##########
    def on_enter_init_task(self):
        print("Initializing Task")
        # success = self.speak("Hello! My name is Tinker. I'm going to welcome the guests!")
        # time.sleep(8)
        
    
    def on_enter_register(self):
        
        # # self.face_register_client.wait_for_server()   #
        # register_req = FaceRegister.Request()
        # register_req.info = ['register']
        # future = self.face_register_client.call_async(register_req)
        # rclpy.spin_until_future_complete(self, future)
        # res = future.result()
        # res: FaceRegisterResponse = self.vision_face_register(self.host_info)
        
        # if res.success:
        #     print("Successfully registered!")
        #     # TODO: CHANGE NAME 
        #     self.speak("Hello, Alex! I successfully register your face!")
        #     time.sleep(10)
        #     self.location_lable = 0
        #     self.active_location = self.locations[self.location_lable]
        #     self.start_move()
        # else:
        #     if res.fail_info == "Cannot detect any faces.":
        #         self.speak("I cannot see host. Could you please stand in front of me?")
        #     elif res.fail_info == "Detect more than one face.":
        #         self.speak("I detect more than one face. Please leave my sight if you are not the host.")
        if self.person_cnt > 0: # Guest
            print("Begin register Guest...")
            # TODO: CHANGE NAME
            # success = self.speak("Hi! Your name is Mike and your favourite drink is milk, right?")
            self.location_lable = 1
        else: # Host
            print("Begin register Host...")
            self.location_lable = 0

        self.active_location = self.locations[self.location_lable]
        self.start_move()
        print("Entered Register")
        
    
    def on_enter_move(self):
        if self.active_location == 'door':
            print("Going to the door.")
            # TODO: ADD NAVIGATION
            # success = self.speak("Hello, I am Tinker. Could you please tell me your name and favourite drink?")
            self.person_cnt += 1
            self.active_person = self.persons[self.person_cnt]
            self.return_to_register()
        else:
            print("Going to sofa.")
            # # TODO: ADD NAVIGATION
            # time.sleep(40)
            # # TODO: Change name
            # self.speak("We arrive the living room. Please stand on my left, Mike!")
            # time.sleep(8)
            self.start_introduce()
        

    def on_enter_introduce(self):
        # success = self.speak("Hello, Alex! This is Mike, and his favourite drink is milk. He is a around 20 years old. He is wearing brown clothes today. His hair is black.")
        # time.sleep(25)
        # success = self.speak("Hello, Mike! This is the host Alex, and his favourite drink is cola. He is a around 25 years old. He is wearing black clothes today. His hair is black.")
        # time.sleep(25)
        # self.speak("Now I am going to the door to welcome next guest.")
        self.start_gesture()

    def on_enter_gesture(self):
        print("Entered Gesture")
        if self.person_cnt < 2:
            self.location_lable = 0
            self.active_location = self.locations[self.location_lable]
            self.return_to_move()
        else:
            self.finish_task()

        # Optionally, loop back to another state or stop here
        # self.start_register() # Uncomment to loop back

    def on_enter_finished(self):
        print("Receptionist Finished")

def main(args=None):
    rclpy.init(args=args)
    receptionist_node = Receptionist()
    rclpy.spin(receptionist_node)
    receptionist_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()