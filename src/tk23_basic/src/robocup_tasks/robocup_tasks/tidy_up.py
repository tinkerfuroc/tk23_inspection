import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from cost_map_msgs.msg import CostMap
from tinker_vision_msgs.msg import Object, Objects
from tinker_vision_msgs.srv import ObjectDetection
from tinker_arm_msgs.srv import URControlService, Robotiq
from audio_common_msgs.action import SoundRequest as SoundRequestAction
from audio_common_msgs.msg import SoundRequest
# from 
import numpy as np
import copy
import time

# This is taken from https://github.com/ros/geometry_tutorials/blob/ros2/turtle_tf2_py/turtle_tf2_py/turtle_tf2_broadcaster.py
# This function is a stripped down version of the code in
# https://github.com/matthew-brett/transforms3d/blob/f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/euler.py
# Besides simplifying it, this version also inverts the order to return x,y,z,w, which is
# the way that ROS prefers it.
import math
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def quaternion_multiply(q0, q1):
    """
    Taken from https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html#id5

    Multiplies two quaternions.

    Input
    :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
    :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)
    """
    # Extract the values from q0
    w0 = q0[0]
    x0 = q0[1]
    y0 = q0[2]
    z0 = q0[3]

    # Extract the values from q1
    w1 = q1[0]
    x1 = q1[1]
    y1 = q1[2]
    z1 = q1[3]

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([q0q1_w, q0q1_x, q0q1_y, q0q1_z])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion


class TidyUp(Node):

    def __init__(self):
        super().__init__('tidy_up')
        self.state = 'init_task'
        self.sub_state = 'call_nav_srv'
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('debug_mode', False)
        self.debug = self.get_parameter('debug_mode').get_parameter_value().bool_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_sub = self.create_subscription(Odometry, 'odometry/filtered', self.odom_callback, 10)
        self.current_pose = None

        # self.costmap_sub = self.create_subscription(Cost)

        # self.navigation_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.navigation_action_client = ActionClient(self, NavigationAction, 'action_server')

        self.detection_client = self.create_client(ObjectDetection, 'object_detection')
        self.detection_future = None
        self.target_objects = []

        self.init_poses = []

        self.target_robot_poses = []    # predefined furniture pose

        self.done_object_ids = set()

        self.navigation_future = None


    def create_msg_header(self, frame):
        return Header(stamp=self.get_clock().now().to_msg(), frame_id=frame)


    def odom_callback(self, odometry):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='odom',
                time=rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'In odom_callback: Could not transform /odom to /map: {ex}')
            return
        
        # odometry = Odometry
        # odometry.pose = PoseWithCovariance
        # odometry.pose.pose = Pose
        self.current_pose = tf2_geometry_msgs.do_transform_pose(odometry.pose.pose, transform)
        # self.get_logger().warn('Update on current_robot_position = %.3f, %.3f, %.3f' % (self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z))


    def navigation_cmd(self, pose: Pose):
        msg = PoseStamped()
        msg.pose = pose
        msg.header = self.create_msg_header('map')
        
        # self.navigation_pub.publish(msg)
        if not self.navigation_action_client.wait_for_server(timeout_sec=0.01):
            return False, None

        return True, self.navigation_action_client.send_goal_async(msg)
    

    def timer_callback(self):
        self.debug = self.get_parameter('debug_mode').get_parameter_value().bool_value
        if self.current_pose is None:
            self.get_logger().warn('Can\'t get current pose')
            return

        if self.state == 'init_task':
            if self.sub_state == 'call_nav_srv':
                # go to the init pose
                self.navigation_future = self.navigation_cmd(self.init_poses[0])

            elif self.sub_state == 'wait_nav_done':
                if self.navigation_future.done():
                    self.state = 'look_around'
                    self.sub_state = 'call_nav_srv'
                    self.grasp_position_stage = 0
            else:
                assert False
        
        elif self.state == 'look_around':
            if self.grasp_position_stage >= 3:
                # finished
                self.state = 'task_done'
                self.sub_state = 'task_done'
                return

            if self.sub_state == 'call_nav_srv':
                # go to the %d-th position
                target = self.target_robot_poses[self.grasp_position_stage]
                success, self.navigation_future = self.navigation_cmd(target)
                if not success:
                    return
                self.sub_state = 'wait_nav_done'

            elif self.sub_state == 'wait_nav_done':
                # wait for navigation done
                if self.navigation_future.done():
                    self.sub_state = 'call_detection_srv'
                
            elif self.sub_state == 'call_detection_srv':
                # find objects
                self.detection_future = self.detection_client.call_async(ObjectDetection.Request())
                self.sub_state = 'wait_detection_return'

            elif self.sub_state == 'wait_detection_return':
                # combine the detection results
                if self.detection_future.done():
                    for o in self.detection_future.result().objects:
                        self.target_objects.append({
                            'conf': o.conf,
                            'name': o.cls,
                            'centroid': o.centroid,
                            'object_id': o.object_id,
                            'similairty': o.similarity
                        })
                    self.sub_state = 'detection_done'

            elif self.sub_state == 'detection_done':
                # move the objects
                self.state = 'move_objects'
                self.sub_state = 'call_grasp_srv'
                max_sim, max_obj = -1, None
                for i in self.target_objects:
                    if i['object_id'] not in self.done_object_ids:
                        if i['similarity'] > max_sim:
                            max_sim = i['similarity']
                            max_obj = i
                
                if max_obj is None or max_sim < 0.3:
                    self.state = 'look_around'
                    self.sub_state = 'call_nav_srv'
                    self.grasp_position_stage += 1

                else:
                    self.target_objects = [i]
                    self.state = 'move_objects'
                    self.sub_state = 'calc_target_pose'

            else:
                assert False

        
        elif self.state == 'move_objects':

            if self.sub_state == 'call_pick_srv':
                # calc pose here
                pass

            if self.sub_state == 'wait_pick_done':
                pass

            if self.sub_state == 'call_nav_srv':
                target = self.target_robot_poses[self.grasp_position_stage]
                success, self.navigation_future = self.navigation_cmd(target)
                if not success:
                    return
                self.sub_state = 'wait_nav_done'

            if self.sub_state == 'wait_nav_done':
                # wait for navigation done
                if self.navigation_future.done():
                    self.sub_state = 'call_place_srv'                

            if self.sub_state == 'call_place_srv':
                # calc pose here
                pass

            if self.sub_state == 'wait_place_done':
                pass
            
            elif self.sub_state == 'grasp_done':
                self.state = 'look_around'
                self.sub_state = 'call_nav_srv'
                self.grasp_position_stage += 1
        
        elif self.state == 'task_done':
            pass

        else:
            assert False



def main(args=None):
    rclpy.init(args=args)

    tidy_up_node = TidyUp()

    _ = input('Press enter to start the test')

    rclpy.spin(tidy_up_node)

    tidy_up_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
