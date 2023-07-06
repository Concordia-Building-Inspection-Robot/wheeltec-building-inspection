#code not good


import paramiko
import threading
import rospy
import time
import socket



def run_command(ssh, command):
    source_ros_cmd = 'source /opt/ros/melodic/setup.bash && source /home/wheeltec/wheeltec_robot/devel/setup.bash && '
    full_command = source_ros_cmd + ' '.join(command)
    stdin, stdout, stderr = ssh.exec_command(full_command)
    output = stdout.read().decode().strip()
    error = stderr.read().decode().strip()
    if error:
        raise Exception(f"Command execution error: {error}")
    return output


def main():

    hostnames = ["192.168.0.100", "192.168.1.101"]
    user = 'wheeltec'
    password = 'dongguan'


    print("Connecting to the big robot with ip: " + hostnames[1])

    #while True:
    try:
        ssh1 = paramiko.SSHClient()
        ssh1.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh1.connect(hostnames[1], username=user, password=password)

        output_1 = run_command(ssh1, ['roslaunch', 'turn_on_wheeltec_robot', 'navigation.launch'])
        print("Output from Robot 2: " + output_1)


    except paramiko.AuthenticationException:
        print("Authentication failed. Please check the SSH credentials.")
    except paramiko.SSHException as ssh_ex:
        print("Unable to establish SSH connection: " + str(ssh_ex))
    except socket.error as sock_ex:
        print("Socket error occurred: " + str(sock_ex))
    except Exception as ex:
        print("An error occurred: " + str(ex))

    #time.sleep(10)
if __name__ == '__main__':
    main()

