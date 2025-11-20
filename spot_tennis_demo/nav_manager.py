#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
import tf_transformations
import tf2_geometry_msgs

class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')

        self.ball = None
        self.bin = None

        self.target_frame = 'spot_nav/map'
        self.base_frame = 'base_link'
        self.hand_frame = 'front_rail'

        self.tf = Buffer()
        TransformListener(self.tf, self)

        self.create_subscription(PoseStamped, '/ball_nav_pose', self.ball_cb, 10)
        self.create_subscription(PoseStamped, '/bin_nav_pose',  self.bin_cb, 10)

        self.create_service(Trigger, '/nav_to_ball', self.nav_ball_cb)
        self.create_service(Trigger, '/nav_to_bin',  self.nav_bin_cb)

        self.nav = ActionClient(self, NavigateToPose, '/spot_nav/navigate_to_pose')

    def ball_cb(self, msg):
        self.ball = msg

    def bin_cb(self, msg):
        self.bin = msg

    def nav_ball_cb(self, req, res):
        return self._go(self.ball, res)
    
    def nav_bin_cb(self, req, res):
        return self._go(self.bin, res)

    def _msg_to_mat(self, msg):
        # Put Poses or Transforms into Matrix Form
        if isinstance(msg, PoseStamped):
            t = msg.pose.position
            q = msg.pose.orientation
        else:
            t = msg.transform.translation
            q = msg.transform.rotation
        return tf_transformations.compose_matrix(
            translate=[t.x, t.y, t.z],
            angles=tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        )
    def _mat_to_pose(self, M, frame):
        # Matrix to Pose
        p = PoseStamped()
        p.header.frame_id = frame
        tr = tf_transformations.translation_from_matrix(M)
        qr = tf_transformations.quaternion_from_matrix(M)
        p.pose.position.x = tr[0]
        p.pose.position.y = tr[1]
        p.pose.position.z = tr[2]
        p.pose.orientation.x = qr[0]
        p.pose.orientation.y = qr[1]
        p.pose.orientation.z = qr[2]
        p.pose.orientation.w = qr[3]
        return p

    def _go(self, hand_pose, res):
        if hand_pose is None:
            res.success = False
            res.message = "No target pose received"
            return res
        
        if not self.nav.wait_for_server(timeout_sec=1.0):
            res.success = False
            res.message = "Nav2 unavailable"
            return res
        
        try:
            if hand_pose.header.frame_id != self.target_frame:
                hand = self.tf.transform(hand_pose, self.target_frame, timeout=Duration(seconds=0.5))
            else:
                hand = hand_pose

        except:
            res.success = False
            res.message = "TF hand to map failed"
            return res
        
        try:
            tf_base_hand = self.tf.lookup_transform(
                self.base_frame, self.hand_frame, Time(), timeout=Duration(seconds=0.5)
            )

        except:
            res.success = False
            res.message = "TF base to hand failed"
            return res

        # Calculate Base Goal Pose (in map frame)
        M_map_hand = self._msg_to_mat(hand)
        M_base_hand = self._msg_to_mat(tf_base_hand)
        M_map_base = M_map_hand @ tf_transformations.inverse_matrix(M_base_hand)
        base_goal = self._mat_to_pose(M_map_base, self.target_frame)

        # Send Nav2 Goal
        goal = NavigateToPose.Goal()
        goal.pose = base_goal
        self.nav.send_goal_async(goal)
        res.success = True
        res.message = "Navigating"
        return res
    
def main(args=None):
    rclpy.init(args=args)
    node = NavManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
