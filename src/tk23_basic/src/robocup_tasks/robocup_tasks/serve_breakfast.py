import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
import tf_transformations

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from geometry_msgs.msg import Transform, TransformStamped, Vector3
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from tinker_vision_msgs.msg import Object, Objects
from tinker_vision_msgs.srv import ObjectDetection
from tinker_arm_msgs.srv import URControlService, Robotiq
from tinker_msgs.srv import GraspnetService
from nav2_msgs.action import NavigateToPose
import numpy as np
import copy
import time


class ServeBreakfastNode(Node):

    def __init__(self):
        super().__init__('carry_my_luggage')
        self.state = 'init_task'
        self.sub_state = 'init_task'

        self.timer_period = 1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.declare_parameter('debug_mode', False)
        self.debug = self.get_parameter('debug_mode').get_parameter_value().bool_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.detection_cli = self.create_client(ObjectDetection, 'object_detection_sevice')
        self.detection_cli_realsense = self.create_client(ObjectDetection, 'object_detection_sevice_realsense')

        self.depth_img_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_img_callback, 10)
        self.bridge = CvBridge()
        self.door_opened = False
        self.door_delay = 2 # seconds

        self.navigation_action_cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.ur_client = self.create_client(URControlService, 'tinker_arm_control_service')
        self.gripper_client = self.create_client(Robotiq, 'robotiq_service')
        self.graspnet_client = self.create_client(GraspnetService, 'graspnet_service')

        # debug publishers
        self.debug_bag_pub = self.create_publisher(PointStamped, 'bag', 10)
        self.debug_target_pose_pub = self.create_publisher(PoseStamped, 'target', 10)

        self.arm_frame_id = 'base_link'

        self.object_name = ['cereal box', 'milk carton', 'bowl', 'spoon']
        self.object_cnt = 0
        self.location_pose = {
            'pick_area': [0, 0, 0, 0],
            'place_area_1': [0, 0, 0, 0],
            'place_area_2': [0, 0, 0, 0],
            'place_area_3': [0, 0, 0, 0],
            'place_area_4': [0, 0, 0, 0],
        }

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
        msg = PoseStamped()
        msg.pose = pose
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        
        if not self.navigation_action_cli.wait_for_server(timeout_sec=0.01):
            return False, None

        return True, self.navigation_action_cli.send_goal_async(msg)
    

    def timer_callback(self):
        self.debug = self.get_parameter('debug_mode').get_parameter_value().bool_value
        print('state =', self.state)

        if self.state == 'init_task':
            if self.sub_state == 'init_task':
                if self.door_opened:
                    if self.door_delay < 1e-6:
                        self.state = 'move_to_intro_area'
                        self.sub_state = 'call_nav_action'
                    else:
                        self.door_delay -= self.timer_period

            else:
                assert False


        elif self.state == 'move_to_pick_area':
            if self.sub_state == 'call_nav_action':
                pose = self.location_pose['pick_area']
                position = Point(x=pose[0], y=pose[1], z=0.0)
                orientation = Pose(x=0.0, y=pose[2], z=0.0, w=pose[3])
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
                    self.state = 'grab_the_bag'
                    self.sub_state = 'pick_objects'

            else:
                assert False

        elif self.state == 'pick_objects':
            if self.sub_state == 'call_detection_srv':
                req = ObjectDetection.Request()
                req.flags = 'match_objects|request_image|request_segments'
                self.detection_future = self.detection_cli_realsense.call_async(req)
                self.sub_state = 'wait_detection_done'

            elif self.sub_state == 'wait_detection_done':
                if self.detection_future.done():
                    result = self.detection_future.result()
                    # find the bag
                    header = result.header

                    sim_max, idx = -1, -1
                    for i, obj in enumerate(result.objects):
                        if obj.object_id == self.object_cnt and obj.similarity > sim_max:
                            sim_max, idx = obj.similarity, i
                    
                    if idx >= 0:
                        print(f'Object {self.object_name[self.object_cnt]} found. Calling graspnet service.')
                        depth_img = result.depth_img
                        segment = result.segments[idx]
                        
                        req = GraspnetService.Request()
                        req.depth_img = depth_img
                        req.segments = [segment]
                        req.source_frame = header.frame_id
                        req.target_frame = 'base_link'

                        self.graspnet_future = self.graspnet_cli.call_async(req)
                        self.sub_state = 'wait_graspnet_done'
            
            elif self.sub_state == 'wait_graspnet_done':
                if self.graspnet_future.done():
                    result = self.graspnet_future.result()
                    # find the pose with highest score
                    max_score, pose = -1, -1
                    for p, s in zip(result.grasp_poses, result.scores):
                        if s > max_score:
                            max_score, pose = s, p

                    if max_score >= 0:
                        # call UR arm service
                        ur_req = URControlService.Request()
                        # ur_req.target_pose = Pose(position=Point(x=0.8,y=-0.5,z=0.3),orientation=Quaternion(x=0.707,y=0.707,z=0.,w=0.))
                        ur_req.target_pose = pose

                        self.arm_future = self.ur_client.call_async(ur_req)
                        self.sub_state = 'wait_arm_done_stage_1'

            elif self.sub_state == 'wait_arm_done_stage_1':
                if self.arm_future.done():
                    # call gripper service
                    gripper_req = Robotiq.Request()
                    gripper_req.distance = 250
                    self.gripper_future = self.gripper_client.call_async(gripper_req)
                    self.sub_state = 'wait_gripper_done_stage_2'

            elif self.sub_state == 'wait_gripper_done_stage_2':
                if self.gripper_future.done():
                    ur_req.target_pose = Pose(position=Point(x=0.11,y=0.4,z=0.6),orientation=Quaternion(x=0.,y=1.,z=0.,w=0.))
                    # ur_req.target_pose = Pose(position=Point(x=0.11,y=0.4,z=0.6),orientation=Quaternion(x=1.,y=0.,z=0.,w=0.))
                    self.arm_future = self.ur_client.call_async(ur_req)
                    self.sub_state = 'wait_arm_done_stage_3'
            
            elif self.sub_state == 'wait_arm_done_stage_3':
                if self.arm_future.done():
                    self.state = 'put_down_objects'
                    self.sub_state = 'call_nav_action'
            
        
        elif self.state == 'put_down_objects':
            if self.sub_state == 'call_nav_action':
                target_pose, arrived = self.get_target_pose(self.current_pose.position, self.person_pos, 0.8)
                if not arrived:
                    success, self.navigation_future = self.navigation_cmd(target_pose)
                    if success:
                        self.sub_state = 'wait_nav_done'
                else:
                    pass
            
            elif self.sub_state == 'wait_nav_done':
                if self.navigation_future.done():
                    self.sub_state = 'call_arm_service'
            
            elif self.sub_state == 'call_arm_service':
                # call UR arm service
                ur_req = URControlService.Request()
                # fixed pose for putting object down
                ur_req.target_pose = Pose(position=Point(x=0.8,y=-0.5,z=0.3),orientation=Quaternion(x=0.707,y=0.707,z=0.,w=0.))

                self.arm_future = self.ur_client.call_async(ur_req)
                self.sub_state = 'wait_arm_done_stage_1'

            elif self.sub_state == 'wait_arm_done_stage_1':
                if self.arm_future.done():
                    # call gripper service
                    gripper_req = Robotiq.Request()
                    gripper_req.distance = 0
                    self.gripper_future = self.gripper_client.call_async(gripper_req)
                    self.sub_state = 'wait_gripper_done_stage_2'

            elif self.sub_state == 'wait_gripper_done_stage_2':
                if self.gripper_future.done():
                    ur_req.target_pose = Pose(position=Point(x=0.11,y=0.4,z=0.6),orientation=Quaternion(x=0.,y=1.,z=0.,w=0.))
                    # ur_req.target_pose = Pose(position=Point(x=0.11,y=0.4,z=0.6),orientation=Quaternion(x=1.,y=0.,z=0.,w=0.))
                    self.arm_future = self.ur_client.call_async(ur_req)
                    self.sub_state = 'wait_arm_done_stage_3'
            
            elif self.sub_state == 'wait_arm_done_stage_3':
                if self.arm_future.done():
                    if self.object_cnt == 3:
                        self.state = 'task_done'
                        self.sub_state = 'task_done'
                    else:
                        self.object_cnt += 1
                        self.state = 'move_to_pick_area'
                        self.sub_state = 'call_nav_action'

        elif self.state == 'task_done':
            print('task done.')

        else:
            # assert False
            pass



def main(args=None):
    rclpy.init(args=args)

    node = ServeBreakfastNode()

    _ = input('Press enter to start the test')

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
