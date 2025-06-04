#! /usr/bin/env python
import os
import rospy
import sys
from rbkairos_navigation.srv import RBKairosPose, RBKairosPoseResponse, RBKairosPoseRequest
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospkg
from shutil import rmtree

class RBKairosPoseRetriever(object):
    def __init__(self, spots_file_name="spots_saved.yaml"):
        self.pose_service = rospy.Service('/record_spot', RBKairosPose , self.pose_service_callback) # create the Service called my_service with the defined callback
        rospy.Subscriber("/robot/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.pose_now = PoseWithCovarianceStamped()

        rospack = rospkg.RosPack()

        # We clean up spots folder
        spots_folder_path = os.path.join(rospack.get_path('rbkairos_navigation'), "spots")
        if os.path.exists(spots_folder_path):
            rmtree(spots_folder_path)

        os.makedirs(spots_folder_path)
        print("Created folder=" + str(spots_folder_path))

        self.spot_file_path = os.path.join(spots_folder_path, spots_file_name)

        try:
            os.remove(self.spot_file_path)
        except:
            rospy.loginfo("File Not Found "+str(self.spot_file_path) )
        # Init File for Spot saving
        file2write=open(self.spot_file_path,'a')
        file2write.write("### This is the Spot Saving File ###\n")
        file2write.close()
        rospy.loginfo("RBKairosPoseRetriever READY...")

    def start_loop_service(self):
        rospy.spin() # mantain the service open.

    def pose_service_callback(self, request):
        spot_name = request.label
        tab_str = "    "
        tab_str2 = "        "
        with open(self.spot_file_path, "a") as myfile:
            spot_pose = self.pose_now.pose.pose
            spot_position = spot_pose.position
            spot_orientation = spot_pose.orientation

            str_spot_pos = tab_str+"position:\n"+tab_str2+"x: "+str(spot_position.x)+"\n"+tab_str2+"y: "+str(spot_position.y)+"\n"+tab_str2+"z: "+str(spot_position.z)+"\n\n"
            str_spot_ori = tab_str+"orientation:\n"+tab_str2+"x: "+str(spot_orientation.x)+"\n"+tab_str2+"y: "+str(spot_orientation.y)+"\n"+tab_str2+"z: "+str(spot_orientation.z)+"\n"+tab_str2+"w: "+str(spot_orientation.w)+"\n\n"

            myfile.write(spot_name+":\n"+str_spot_pos+str_spot_ori)

        response = RBKairosPoseResponse()
        response.navigation_successfull = True
        response.message = "Spot <"+spot_name+"> Saved"

        return response

    def amcl_pose_callback(self, message):
        self.pose_now = message



def startPoseService():
    rospy.init_node('pose_service_server', log_level=rospy.INFO)
    if len(sys.argv) < 2:
        print("usage: spots_to_file.py spots_file_name")
    else:
        spots_file_name = str(sys.argv[1])+".yaml"

        rbkairos_pose_rtrv = RBKairosPoseRetriever(spots_file_name=spots_file_name)
        rbkairos_pose_rtrv.start_loop_service()

if __name__ == "__main__":
    startPoseService()
