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

ssh = SSHClient()
ssh.load_system_host_keys()
ssh.connect('192.168.8.1', username = 'root', password = 'oelinux123')
print('connected')

scp = SCPClient(ssh.get_transport())
scp.get('/home/root/tag_output.txt', '/Users/noahfisher/april_tag/')

print('success')
scp.close() 
