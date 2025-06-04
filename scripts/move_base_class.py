#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import rosparam


class SendCoordinates(object):
    def __init__(self, goal_labels, run_once=True):
        self.goal_labels = goal_labels
        self.run_once = run_once
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rospy.on_shutdown(self.shutdownhook)
        
    def send_all_goals(self):
        loop = True
        while loop and not rospy.is_shutdown():
            for label in self.goal_labels:
                success = self.sendGoal(label)
                if not success:
                    rospy.logwarn("Goal failed: %s", label)
                if rospy.is_shutdown():
                    break
            loop = not self.run_once

    def sendGoal(self, label):
        goal=MoveBaseGoal()
        goal_tmp = Pose()

        self._ctrl_c = False
        
        while not self._ctrl_c:

            goal_tmp.position.x=rosparam.get_param(label+'/position/x')
            goal_tmp.position.y=rosparam.get_param(label+'/position/y')
            goal_tmp.position.z=rosparam.get_param(label+'/position/z')
            goal_tmp.orientation.x=rosparam.get_param(label+'/orientation/x')
            goal_tmp.orientation.y=rosparam.get_param(label+'/orientation/y')
            goal_tmp.orientation.z=rosparam.get_param(label+'/orientation/z')
            goal_tmp.orientation.w=rosparam.get_param(label+'/orientation/w')

            goal.target_pose.pose=goal_tmp
            goal.target_pose.header.frame_id='map'
            goal.target_pose.header.stamp = rospy.Time.now()

            self.client.wait_for_server()
            rospy.loginfo('Going to spot='+str(label))
            self.client.send_goal(goal, feedback_cb=self.callback)
            self.client.wait_for_result()
            result=self.client.get_state()

            #print result
            if result==3:
                print('successfuly reached point: '+str(label))
                return True


    def shutdownhook(self):
        rospy.loginfo("Shutdown signal received. Cancelling goal.")
        self.client.cancel_all_goals()
        self._ctrl_c = True

    def callback(self, data):
        return


if __name__ == "__main__":
    rospy.init_node('send_coordinates_node', log_level=rospy.INFO)

    # Load from ROS param, or fallback to hardcoded values
    goal_list = rospy.get_param("~goal_labels", ["front_of_lab_door", "start_pos_lab"])
    run_once = rospy.get_param("~run_once", True)

    sender = SendCoordinates(goal_list, run_once=run_once)
    sender.send_all_goals()
