#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
import tf


class AStarReplanner:
    def __init__(self):
        rospy.init_node('astar_replanner')

        self.listener = tf.TransformListener()
        self.current_goal = None

        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback)

        rospy.wait_for_service('/move_base/make_plan')
        self.make_plan_srv = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

        rospy.Timer(rospy.Duration(0.2), self.replan_callback)  # 2 Hz

    def goal_callback(self, msg):
        self.current_goal = msg.goal.target_pose
        self.current_goal.header.frame_id = "odom"  # enforce odom frame

    def get_robot_pose(self):
        try:
            now = rospy.Time(0)
            self.listener.waitForTransform("odom", "base_link", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("odom", "base_link", now)

            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.header.stamp = now
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]
            return pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s", e)
            return None

    def replan_callback(self, event):
        if self.current_goal is None:
            rospy.logwarn_throttle(10, "No goal received yet...")
            return

        start = self.get_robot_pose()
        if start is None:
            return

        try:
            plan_resp = self.make_plan_srv(start=start, goal=self.current_goal, tolerance=0.5)
            if plan_resp.plan.poses:
                rospy.loginfo_throttle(2, "A* replanned with %d points", len(plan_resp.plan.poses))
            else:
                rospy.logwarn("A* planner returned empty path")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to call make_plan: %s", e)


if __name__ == '__main__':
    try:
        AStarReplanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
