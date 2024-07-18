import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
import tf_transformations

from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from geometry_msgs.msg import Transform, TransformStamped, Vector3
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from tinker_vision_msgs.msg import Object, Objects
from tinker_vision_msgs.srv import ObjectDetection
from tinker_arm_msgs.srv import URControlService, Robotiq
from tinker_msgs.srv import GraspnetService
from audio_common_msgs.action import SoundRequest as SoundRequestAction
from audio_common_msgs.msg import SoundRequest
from nav2_msgs.action import NavigateToPose
import numpy as np
import copy
import time


# helper functions
def get_time_sec(stamp):
    if isinstance(stamp, rclpy.time.Time):
        return (stamp.nanoseconds // 1e6) / 1e3 if stamp else None
    assert False


class CarryMyLuggage(Node):

    def __init__(self):
        super().__init__('carry_my_luggage')
        self.state = 'init_task'
        self.sub_state = 'init_task'

        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('debug_mode', False)
        self.debug = self.get_parameter('debug_mode').get_parameter_value().bool_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.detection_sub = self.create_subscription(
            Objects,
            'detection_publisher',
            self.detection_callback,
            10)
        self.detection_result = None

        self.odom_sub = self.create_subscription(Odometry, 'odometry/filtered', self.odom_callback, 10)
        self.current_pose = None

        self.navigation_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        self.detection_cli = self.create_client(ObjectDetection, 'object_detection_sevice')
        self.detection_cli_realsense = self.create_client(ObjectDetection, 'object_detection_sevice_realsense')

        self.navigation_action_cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.ur_client = self.create_client(URControlService, 'tinker_arm_control_service')
        self.gripper_client = self.create_client(Robotiq, 'robotiq_service')
        self.graspnet_client = self.create_client(GraspnetService, 'graspnet_service')

        self.audio_client = ActionClient(self, SoundRequestAction, 'sound_play')

        # debug publishers
        self.debug_bag_pub = self.create_publisher(PointStamped, 'bag', 10)
        self.debug_person_pub = self.create_publisher(PointStamped, 'person', 10)
        self.debug_target_pose_pub = self.create_publisher(PoseStamped, 'tpose', 10)
        
        self.person_pos = None
        self.person_pos_update_time = None

        self.bag_pos = None
        
        self.initial_pose = None

        self.arm_future = None
        self.client_list = []

        # Test
        # p1 = self.create_publisher(PointStamped, 'p1', 10)
        # p2 = self.create_publisher(PointStamped, 'p2', 10)
        # p3 = self.create_publisher(TFMessage, '/tf', 10)
        # while True:
        #     header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        #     # a = [4.0, -1.0, 2.0, 4.0, 1.0, 2.0]
        #     a = (np.random.rand(6) - 0.5) * 3
        #     x = Point(x=a[0], y=a[1], z=a[2])
        #     y = Point(x=a[3], y=a[4], z=a[5])
        #     p1.publish(PointStamped(header=header, point=x))
        #     p2.publish(PointStamped(header=header, point=y))
        #     p, _ = self.get_target_pose(x, y, 0.6)
        #     tr = Transform(translation=Vector3(x=p.position.x, y=p.position.y, z=p.position.z), rotation=p.orientation)
        #     # tr = Transform(translation=Vector3(x=.5, y=.5, z=.5), rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
        #     tf_msg = TFMessage()
        #     tf_msg.transforms = [TransformStamped(header=header, child_frame_id='f', transform=tr)]
        #     p3.publish(tf_msg)
        #     time.sleep(4)

        self.get_logger().info('Node initialized.')

        # self.bag_pose = None

    
    def odom_callback(self, odometry):
        # type() = Pose
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='odom',
                time=rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform /odom to /map: {ex}')
            return
        
        # odometry = Odometry
        # odometry.pose = PoseWithCovariance
        # odometry.pose.pose = Pose
        self.current_pose = tf2_geometry_msgs.do_transform_pose(odometry.pose.pose, transform)
        # self.get_logger().warn('Update on current_robot_position = %.3f, %.3f, %.3f' % (self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z))


    def detection_callback(self, objects_msg):
        if objects_msg.status == 0:
            self.detection_result = objects_msg
            self.detection_result_time = get_time_sec(self.get_clock().now())
            for obj in objects_msg.objects:
                if obj.id > 0:
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            target_frame='map',
                            target_time=rclpy.time.Time(),
                            source_frame=objects_msg.header.frame_id,
                            source_time=rclpy.time.Time.from_msg(objects_msg.header.stamp),
                            fixed_time='map'
                        )
                    except TransformException as ex:
                        self.get_logger().error(f'Could not transform {objects_msg.header.frame_id} to /map: {ex}')
                        return
                    
                    self.person_pos = transform.do_transform_point(PointStamped(
                        header=objects_msg.header,
                        point=obj.centroid
                    ))
                    self.person_pos_update_time = get_time_sec(self.get_clock().now())
    

    def navigation_cmd(self, pose: Pose):
        msg = PoseStamped()
        msg.pose = pose
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        
        if not self.navigation_action_cli.wait_for_server(timeout_sec=0.03):
            return False, None

        return True, self.navigation_action_cli.send_goal_async(msg)
    

    def speak(self, content):
        goal_msg = SoundRequestAction.Goal()
        msg = SoundRequest()
        msg.sound = SoundRequest.SAY
        msg.command = SoundRequest.PLAY_ONCE
        msg.volume = 0.5
        msg.arg = content
        goal_msg.sound_request = msg

        self.audio_client.wait_for_server()

        success = self.audio_client.send_goal_async(goal_msg)

        return True


    def get_target_pose(self, start: Point, end: Point, dist: float, flatten=False) -> Pose:
        '''
        Get the target pose from start (x0, y0, z0) to end (x1, y1, z1) s.t.
        - distance(pose.position, (x1, y1, z1)) == dist
        - pose.orientation = (x1, y1, z1) - (x0, y0, z0)
        Returns: (Pose, arrived)
        '''
        x0, y0, z0 = start.x, start.y, start.z
        x1, y1, z1 = end.x, end.y, end.z

        if flatten:
            z0, z1 = 0, 0

        d_st = np.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2 + (z0 - z1) ** 2)
        if d_st < dist + 1e-6:
            position = start
        else:
            alpha = dist / d_st
            position = Point(x=x0 * alpha + x1 * (1 - alpha),
                             y=y0 * alpha + y1 * (1 - alpha),
                             z=z0 * alpha + z1 * (1 - alpha))
        
        rotation = np.eye(4)
        # (x0, y0, z0) ------> (x1, y1, z1)
        x, y, z = x1 - x0, y1 - y0, z1 - z0
        # x-axis
        rotation[:3, 0] = np.array([x, y, z]) / np.sqrt(x ** 2 + y ** 2 + z ** 2)
        # z-axis, must be (0, 0, 1) when z == 0
        rotation[:3, 2] = np.array([-x * z, - y * z, x ** 2 + y ** 2]) / np.sqrt((x ** 2 + y ** 2 + z ** 2) * (x ** 2 + y ** 2))
        # y-axis, calculated from x and z
        rotation[:3, 1] = np.array([-y, x, 0]) / np.sqrt(x ** 2 + y ** 2)
        
        q = tf_transformations.quaternion_from_matrix(rotation)

        return Pose(position=position, orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])), d_st < dist + 1e-6
    

    def point_distance(x: Point, y: Point):
        return np.sqrt((x.x - y.x) ** 2 + (x.y - y.y) ** 2 + (x.z - y.z) ** 2)

    def timer_callback(self):
        self.debug = self.get_parameter('debug_mode').get_parameter_value().bool_value
        if self.current_pose is None:
            self.get_logger().warn('Can\'t get current pose')
            return
        print('state =', self.state)

        if self.state == 'init_task':
            if self.sub_state == 'init_task':
                # self.speak("Hello! My name is Tinker. What can I help for you?")
                self.initial_pose = self.current_pose
                self.state = 'find_bag'
                self.sub_state = 'call_detection_srv'

            else:
                assert False


        elif self.state == 'identify_bag':
            if self.sub_state == 'call_detection_srv':
                # send object detection request
                req = ObjectDetection.Request()
                req.flags = 'find_pointed_object'
                self.detection_future = self.detection_cli.call_async(req)
                self.sub_state = 'wait_detection_done'
            
            elif self.sub_state == 'wait_detection_done':
                if self.detection_future.done():
                    header = self.detection_future.result().header
                    for obj in self.detection_future.result().objects:
                        if obj.being_pointed > 0:
                            # transform the centroid to /map frame
                            try:
                                transform = self.tf_buffer.lookup_transform(
                                    target_frame='map',
                                    source_frame=header.frame_id,
                                    time=rclpy.time.Time()
                                )
                            except TransformException as ex:
                                self.get_logger().error(f'Could not transform {header.frame_id} to /map: {ex}')
                                return
                            
                            self.bag_pos = tf2_geometry_msgs.do_transform_point(PointStamped(header=header, point=obj.centroid), transform)
                            # debug
                            self.debug_bag_pub.publish(self.bag_pos)

                            self.state = 'move_to_bag'
                            self.sub_state = 'call_nav_action'


        elif self.state == 'move_to_bag':
            if self.sub_state == 'call_nav_action':
                target_pose, arrived = self.get_target_pose(self.current_pose.position, self.bag_pos.point, 0.5, flatten=True)
                # debug
                self.debug_target_pose_pub.publish(target_pose)
                
                if not arrived:
                    # call navigation action
                    success, self.navigation_goal_future = self.navigation_cmd(target_pose)
                    if success:
                        self.sub_state = 'wait_nav_goal_response'
                else:
                    # skip this state
                    self.state = 'grab_the_bag'
                    self.sub_state = 'call_detection_srv'
            
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
                    self.sub_state = 'call_detection_srv'

            else:
                assert False
        

        elif self.state == 'grab_the_bag':
            if self.sub_state == 'call_detection_srv':
                req = ObjectDetection.Request()
                req.flags = 'request_image|request_segments'
                self.detection_future = self.detection_cli_realsense.call_async(req)
                self.sub_state = 'wait_detection_done'

            elif self.sub_state == 'wait_detection_done':
                if self.detection_future.done():
                    result = self.detection_future.result()
                    # find the bag
                    header = result.header
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            target_frame='map',
                            source_frame=header.frame_id,
                            time=rclpy.time.Time()
                        )
                    except TransformException as ex:
                        self.get_logger().error(f'Could not transform {header.frame_id} to /map: {ex}')
                        return
                    
                    min_dis, idx = 1e6, -1
                    for i, obj in enumerate(result.objects):
                        cent = tf2_geometry_msgs.do_transform_point(PointStamped(header=header, point=obj.centroid), transform)
                        dist = self.point_distance(cent.point, self.bag_pos.point)
                        if dist < min_dis:
                            min_dis, idx = dist, i
                    
                    if idx >= 0:
                        print('Bag found. Calling graspnet service.')
                        depth_img = result.depth_img
                        segment = result.segments[idx]
                        
                        req = GraspnetService.Request()
                        req.depth_img = depth_img
                        req.segments = [segment]
                        req.source_frame = header.frame_id
                        req.target_frame = 'map'

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
                        # ur_req.target_pose = Pose(position=bag_point.point,orientation=Quaternion(x=1.,y=0.,z=0.,w=0.))
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
                    self.state = 'follow_the_person'
                    self.sub_state = 'call_nav_action'
            
        
        elif self.state == 'follow_the_person':
            if self.sub_state == 'call_nav_action':
                target_pose, arrived = self.get_target_pose(self.current_pose.position, self.person_pos.point, 0.8, flatten=True)
                # debug
                self.debug_target_pose_pub.publish(target_pose)

                if not arrived:
                    # call navigation action
                    success, self.navigation_goal_future = self.navigation_cmd(target_pose)
                    if success:
                        self.sub_state = 'wait_nav_goal_response'
                else:
                    pass
            
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
                    self.sub_state = 'call_nav_action'
                    

        elif self.state == 'put_down_the_bag':
            self.get_logger().warn('Putting down the bag...')

            if len(self.client_list) == 3:
                ur_req = URControlService.Request()
                ur_req.target_pose = Pose(position=Point(x=0.7,y=0.1,z=0.9),orientation=Quaternion(x=1.,y=0.,z=0.,w=0.))
                arm_future = self.ur_client.call_async(ur_req)
                self.client_list.append(arm_future)

            if len(self.client_list) == 4:
                if self.client_list[3].done():
                    gripper_req = Robotiq.Request()
                    gripper_req.distance = 0
                    gripper_future = self.gripper_client.call_async(gripper_req)
                    self.client_list.append(gripper_future)
                else:
                    self.get_logger().warn('Stage 1 running...')
            
            if len(self.client_list) == 5:
                if self.client_list[4].done():
                    self.state = "go_back"
                    self.get_logger().warn('Done, getting back...')
                else:
                    self.get_logger().warn('Stage 2 running...')
            

        elif self.state == "go_back":
            self.navigation_cmd(self.initial_pose)

        else:
            # assert False
            pass



def main(args=None):
    rclpy.init(args=args)

    carry_my_luggage_node = CarryMyLuggage()

    _ = input('Press enter to start the test')

    rclpy.spin(carry_my_luggage_node)

    carry_my_luggage_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
