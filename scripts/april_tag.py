'''
 This Program is used to get the position of an april tag relative
 to the drone from a file located on the remote server (drone). This 
 file is copied to the local machine, parsed to obtain the relevant
 prameters, and published to a ROS node.
 
 Outline:
 1. Connect to remote server (drone)
 2. SCP file to local host
 3. Parse text file to obtain relevant parameters
 4. Publish to ROS 
'''
from paramiko import SSHClient
from scp import SCPClient
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

ssh = SSHClient()
ssh.load_system_host_keys()
ssh.connect('192.168.8.1', username = 'root', password = 'oelinux123')
print('connected')

scp = SCPClient(ssh.get_transport())
scp.get('/home/root/tag_output.txt', '/Users/elissaito/Documents/16.84')

print('success')
f = open("tag_output.txt", "r")
firstline = f.readline()
secondline = f.readline()
thirdline = f.readline()
xyz = secondline.split("  ") #adjust this depending on spacing
rpy = thirdline.split("  ") #adjust this depending on spacing

def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

rpy[3] = float(rpy[3])
quaternion = euler_to_quaternion(rpy[1:4])

def talker():
    rospy.sleep(5)
    pose_pub = rospy.Publisher("chatter", PoseStamped)
    rospy.init_node("talker")
    rospy.sleep(2)
    while not rospy.is_shutdown():
        april = PoseStamped()
        april.header.frame_id = "apriltag_pose"
        april.header.stamp = rospy.Time.now()
        april.pose.position.x = xyz[1]
        april.pose.position.y = xyz[2]
        april.pose.position.z = float(xyz[3])
        april.pose.orientation.x = quaternion[0]
        april.pose.orientation.y = quaternion[1]
        april.pose.orientation.z = quaternion[2]
        april.pose.orientation.w = quaternion[3]
        pose_pub.publish(april)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

scp.close() 
