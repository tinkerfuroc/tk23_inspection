import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Quaternion
from geometry_msgs.msg import Transform, TransformStamped, Vector3, Twist
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
# from tts_interfaces.action import TTSAction
from nav2_msgs.action import NavigateToPose
import numpy as np
import copy
import time


class InspectionNode(Node):

    def __init__(self):
        super().__init__('inspection')
        self.state = 'init_task'
        self.sub_state = 'init_task'

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.depth_img_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_img_callback, 10)
        self.bridge = CvBridge()
        self.door_opened = False
        self.door_delay = 2 # seconds

        self.navigation_action_cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # self.tts_action_cli = ActionClient(self, TTSAction, 'tts_command')

        self.location_pose = {
            'intro_area': [-0.5410987138748169, -1.3992562294006348, -0.37233629734770607, 0.9280978836725147],
            'exit': [3.945643186569214, 2.3515737056732178, 0.03946791694782974, 0.9992208382193594],
        }
        self.waypoints = [
            # [-1.269416093826294, 0.6159919500350952, -0.6035668727662721, 0.7973123792461414],
            # [0.057959675788879395, -2.0434014797210693, -0.37233629734770607, 0.9280978836725147],
            # [1.929534673690796, -2.3699305057525635, 0.7406563259398454, 0.6718840724747756],    
            # [1.8358533382415771, -0.7332605123519897, 0.6602278060143641, 0.7510654060502715],
            # [1.9305953979492188, 1.67796790599823, 0.7480098481294764, 0.6636876276542434],
            # [4.153799533843994, 2.2744040489196777, 0.1026027940263938, 0.9947224068341768],
            [-2.539687156677246, 0.23555725812911987, -1.0, 0.0],
            [-5.191521167755127, -0.22889000177383423, -0.9746243008023739, 0.2238469840884253],
            [-4.183984756469727, -3.051917314529419, 0.007537231665645774, 0.9999715946659777],
            [-2.5281481742858887, -4.619701385498047, -0.7962566459588354, 0.6049589686634838],
        ]
        self.waypoint_cnt = 0
        self.waypoint_target = 1
        # position = (pose[0], pose[1], 0), orientation = (0, 0, pose[2], pose[3])

        # self.initial_pose = [0.7865956425666809, -0.3229368031024933, 0.9999406575538987, 0.010894097974434204]
        self.initial_pose = [-0.113404357, -0.3229368031024933, 1.0, 0.0]
        
        self.init_pos_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'tinker_controller/cmd_vel_unstamped', 10)
        self.walk_timer_period = 0.01
        self.walk_timer = self.create_timer(self.walk_timer_period, self.walk_timer_callback)
        self.walk_flag = False
        self.walk_cnt = 0
        self.wait_init_cnt = 0

        self.get_logger().info('Node initialized.')

    
    def depth_img_callback(self, msg):
        if self.state == 'init_task':
            depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            depth_img = depth_img.astype(float) / 1000.0
            center = depth_img[depth_img.shape[0] // 2, depth_img.shape[1] // 2]
            print('center = ', center)
            if center > 1.8 or center < 1e-6:
                self.door_opened = True


    def navigation_cmd(self, pose: Pose):
        goal = NavigateToPose.Goal()
        msg = PoseStamped()
        msg.pose = pose
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        goal.pose = msg
        
        if not self.navigation_action_cli.wait_for_server(timeout_sec=0.01):
            return False, None

        return True, self.navigation_action_cli.send_goal_async(goal)


    def walk_timer_callback(self):
        if self.walk_flag:
            msg = Twist(linear=Vector3(x=0.3, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
            self.cmd_vel_pub.publish(msg)
    

    def timer_callback(self):
        # self.debug = self.get_parameter('debug_mode').get_parameter_value().bool_value
        print(f'state = {self.state}, moving to waypoint {self.waypoint_cnt}')

        if self.state == 'init_task':
            if self.sub_state == 'init_task':
                if self.door_opened:
                    if self.door_delay < 1e-6:
                        self.state = 'walk'
                    else:
                        self.door_delay -= self.timer_period

            else:
                assert False

        elif self.state == 'walk':
            self.walk_flag = True
            self.walk_cnt += 1
            if self.walk_cnt * self.timer_period > 3:
                self.walk_flag = False

                pose = self.initial_pose
                position = Point(x=pose[0], y=pose[1], z=0.0)
                orientation = Quaternion(x=0.0, y=0.0, z=pose[2], w=pose[3])
                pose_with_cov = PoseWithCovariance(pose=Pose(position=position, orientation=orientation), covariance=[0.0] * 36)

                msg = PoseWithCovarianceStamped()
                msg.pose = pose_with_cov
                msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='odom')
                self.init_pos_pub.publish(msg)

                self.state = 'wait_for_init_pose'

        elif self.state == 'wait_for_init_pose':
            self.wait_init_cnt += 1
            if self.wait_init_cnt * self.timer_period > 1:
                self.state = 'move_to_intro_area'
                self.sub_state = 'call_nav_action'


        elif self.state == 'move_to_intro_area':
            if self.sub_state == 'call_nav_action':
                # pose = self.location_pose['intro_area']
                pose = self.waypoints[self.waypoint_cnt]
                position = Point(x=pose[0], y=pose[1], z=0.0)
                orientation = Quaternion(x=0.0, y=0.0, z=pose[2], w=pose[3])
                # call navigation action
                success, self.navigation_goal_future = self.navigation_cmd(Pose(position=position, orientation=orientation))
                if success:
                    self.sub_state = 'wait_nav_goal_response'
            
            elif self.sub_state == 'wait_nav_goal_response':
                if self.navigation_goal_future.done():
                    goal_handle = self.navigation_goal_future.result()
                    if goal_handle.accepted:
                        self.navigation_result_future = goal_handle.get_result_async()
                        self.sub_state = 'wait_nav_result'
                    else:
                        self.sub_state = 'call_nav_action'
            
            elif self.sub_state == 'wait_nav_result':
                if self.navigation_result_future.done():
                    # self.state = 'self_intro'
                    # self.sub_state = 'call_tts_action'
                    self.waypoint_cnt += 1
                    if self.waypoint_cnt > self.waypoint_target:
                        self.state = 'self_intro'
                        self.sub_state = 'call_tts_action'
                    else:
                        self.sub_state = 'call_nav_action'
            
            else:
                assert False

        elif self.state == 'self_intro':
            _ = input('Press enter to continue')
            self.state = 'move_to_exit'
            self.sub_state = 'call_nav_action'
            # if self.sub_state == 'call_tts_action':
            #     # call tts action
            #     success, self.tts_goal_future = self.tts_cmd('Hello, I am Tinker. I am here to serve you breakfast.')
            #     if success:
            #         self.sub_state = 'wait_tts_goal_response'
            
            # elif self.sub_state == 'wait_tts_goal_response':
            #     if self.tts_goal_future.done():
            #         goal_handle = self.tts_goal_future.result()
            #         if goal_handle.accepted:
            #             self.tts_result_future = goal_handle.get_result_async()
            #             self.sub_state = 'wait_tts_result'
            #         else:
            #             self.sub_state = 'call_tts_action'
            
            # elif self.sub_state == 'wait_tts_result':
            #     if self.tts_result_future.done():
            #         self.state = 'move_to_exit'
            #         self.sub_state = 'call_nav_action'

            # else:
            #     assert False


        elif self.state == 'move_to_exit':
            if self.sub_state == 'call_nav_action':
                # pose = self.location_pose['exit']
                pose = self.waypoints[self.waypoint_cnt]
                position = Point(x=pose[0], y=pose[1], z=0.0)
                orientation = Quaternion(x=0.0, y=0.0, z=pose[2], w=pose[3])
                # call navigation action
                success, self.navigation_goal_future = self.navigation_cmd(Pose(position=position, orientation=orientation))
                if success:
                    self.sub_state = 'wait_nav_goal_response'
            
            elif self.sub_state == 'wait_nav_goal_response':
                if self.navigation_goal_future.done():
                    goal_handle = self.navigation_goal_future.result()
                    if goal_handle.accepted:
                        self.navigation_result_future = goal_handle.get_result_async()
                        self.sub_state = 'wait_nav_result'
                    else:
                        self.sub_state = 'call_nav_action'
            
            elif self.sub_state == 'wait_nav_result':
                if self.navigation_result_future.done():
                    # self.state = 'task_done'
                    # self.sub_state = 'task_done'
                    self.waypoint_cnt += 1
                    if self.waypoint_cnt >= len(self.waypoints):
                        self.state = 'task_done'
                        self.sub_state = 'task_done'
                    else:
                        self.sub_state = 'call_nav_action'

            else:
                assert False

        elif self.state == 'task_done':
            print('done.')

        else:
            # assert False
            pass



def main(args=None):
    rclpy.init(args=args)

    node = InspectionNode()

    _ = input('Press enter to start the test')

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
