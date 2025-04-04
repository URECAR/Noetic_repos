#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ur_dashboard_msgs.srv import GetProgramState
from ur_dashboard_msgs.msg import ProgramState
import tf.transformations
import threading
from math import pi, tau, dist, fabs, cos,sqrt
import binascii
import serial
from moveit_commander.conversions import pose_to_list
import time
import socket
import math
import signal

group_name = "manipulator"
isTest = True
TABLE_EXT_HEIGHT = 500
#### Pose List ###
posj_Zero_pose = [0,-90,90,-90,-90,0]
posx_Table_high = [-706.8, -55.80, 550.0, -3.14, 0.00, -1.58]

posx_Obj1 = [-707, -211, 416.5, -3.14, 0.00, -1.57]
posx_Obj2 = [-706, -64, 416.5, -3.14, 0.00, -1.58]
posx_Obj3 = [-706, 82, 416.5, -3.14, 0.00, -1.57]
posx_Obj1_high = posx_Obj1.copy()  # 원래 값을 유지하려면 복사본을 사용
posx_Obj1_high[2] = TABLE_EXT_HEIGHT
posx_Obj2_high = posx_Obj2.copy()  # 원래 값을 유지하려면 복사본을 사용
posx_Obj2_high[2] = TABLE_EXT_HEIGHT
posx_Obj3_high = posx_Obj3.copy()  # 원래 값을 유지하려면 복사본을 사용
posx_Obj3_high[2] = TABLE_EXT_HEIGHT

#### Base Variables ##############
Cartesian_Speed = 0.2   # m/s
Max_Joint_VEL   = 0.3   # rad/s
Max_Joint_ACC   = 0.3   # rad/s^2

Server_IP = '127.0.0.1'
Server_Port = 5000
Rate = 10   # State 확인 주기(hz)
#### Init Variables ##############
isInterrupted = False
UR = None

class Move_Group(object):
    def __init__(self):
        super(Move_Group, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_ur", anonymous=True)
        self.robot = moveit_commander.RobotCommander()                          # moveit commander Init
        self.scene = moveit_commander.PlanningSceneInterface()                  # moveit world spawn함
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.box_name = ""
        self.current_joints = None
        self.current_pose = None
        self.adjusted_pose = None
        self.callback_count = 0
        self.is_stopped = None
        self.move_group.set_max_velocity_scaling_factor(Max_Joint_VEL)
        self.move_group.set_max_acceleration_scaling_factor(Max_Joint_ACC)
        self.move_group.limit_max_cartesian_link_speed(Cartesian_Speed)
        rospy.wait_for_service('/ur_hardware_interface/dashboard/program_state')
        self.SRV_program_state = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
        self.program_state = None   # 펜던트의 프로그램 실행 상태
        self.is_stopped = False     # 펜던트에서 Emergency Stop이나 프로그램 정지 상태일 시 True
        print("\033[1;33m============")
        print("초기화 완료")
        print("============\033[0m")

        self.ui_thread = threading.Thread(target=self.update_current_state)
        self.ui_thread.daemon = True    # 메인 스레드 종료 시 같이 서브 스레드도 종료
        self.ui_thread.start()

        rospy.sleep(0.1)

    def update_current_state(self):
        rate = rospy.Rate(Rate)
        while not rospy.is_shutdown():

            self.current_joints = [x * (360.0 / tau) for x in self.move_group.get_current_joint_values()]
            self.current_pose = self.move_group.get_current_pose().pose
            current_rot = tf.transformations.euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])
            self.adjusted_pose = [self.current_pose.position.x * -1000, self.current_pose.position.y * -1000, self.current_pose.position.z * 1000, current_rot[0], current_rot[1], current_rot[2]]

            program_state_response = self.SRV_program_state()
            if not self.is_stopped and not isInterrupted:
                self.program_state = program_state_response.state.state
            self.is_stopped = (self.program_state == "STOPPED")
            if self.is_stopped:
                print("\033[1;31mUR 제어에 실패함. \033[0;32m펜던트 내 프로그램 실행 여부를 확인해주십시오.\033[0m")
                break

            if self.callback_count > 10:

                # print("\nCurrent Posx : [{:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}]".format(*self.adjusted_pose))
                # print("Current Posj : [{:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}]".format(*self.current_joints))
                # print(f"Program State: {self.program_state}")
                self.callback_count -= 10
            self.callback_count += 1
            rate.sleep()
        time.sleep(0.2)
        print("상태 업데이트를 정지합니다.")

    def add_obstacle(self, obj_name, coords, rel=False):
        if self.is_stopped:
            return
        if len(coords) != 6:
            rospy.logerr(f"장애물'{obj_name}'의 인자는 6개여야 하지만 {len(coords)}개를 받음.")
            return
        
        # 모든 좌표를 미터 단위로 변환
        coords = [coord * 0.001 for coord in coords]
        
        if rel:
            x, y, z, xsize, ysize, zsize = coords
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = "base_link"
            box_pose.pose.position.x = -x
            box_pose.pose.position.y = -y
            box_pose.pose.position.z = z
            box_pose.pose.orientation.w = 1.0
            box_size = (xsize, ysize, zsize)
        else:            
            x1, y1, z1, x2, y2, z2 = coords
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = "base_link"
            box_pose.pose.position.x = (x1 + x2) / 2
            box_pose.pose.position.y = (y1 + y2) / 2
            box_pose.pose.position.z = (z1 + z2) / 2
            box_pose.pose.orientation.w = 1.0
            box_size = (abs(x2 - x1), abs(y2 - y1), abs(z2 - z1))

        self.scene.add_box(obj_name, box_pose, size=box_size)
        rospy.loginfo(f"장애물 '{obj_name}' 생성 완료.")
        rospy.sleep(0.1)

    def movej(self, add_joints, Rel=False):
        if self.is_stopped:
            return
        if Rel:
            joint_goal = [current_joint + (tau / 360) * add_joint for current_joint, add_joint in zip(self.current_joints, add_joints)]
        else:
            joint_goal = [(tau / 360) * add_joint for add_joint in add_joints]

        success = self.move_group.go(joint_goal, wait=True)
        if not success:
            print("Normal Move Failed!")
        self.move_group.stop()

    def movel(self, waypoints, mod="None"):
        if self.is_stopped:
            return
        waypoints_list = []
        wpose = self.move_group.get_current_pose().pose
        if mod == "rel":
            for waypoint in waypoints:
                wpose_copy = copy.deepcopy(wpose)
                wpose_copy.position.x += waypoint[0] * -0.001  # 밀리미터 단위로 이동
                wpose_copy.position.y += waypoint[1] * -0.001
                wpose_copy.position.z += waypoint[2] * 0.001
                current_rot = tf.transformations.euler_from_quaternion([wpose_copy.orientation.x, wpose_copy.orientation.y, wpose_copy.orientation.z, wpose_copy.orientation.w])
                new_rot = [current_rot[i] + waypoint[i + 3] for i in range(3)]
                new_quat = tf.transformations.quaternion_from_euler(*new_rot)
                wpose_copy.orientation.x, wpose_copy.orientation.y, wpose_copy.orientation.z, wpose_copy.orientation.w = new_quat
                waypoints_list.append(wpose_copy)
        elif mod == "abs":
            for waypoint in waypoints:
                wpose_copy = copy.deepcopy(wpose)
                wpose_copy.position.x = waypoint[0] * -0.001
                wpose_copy.position.y = waypoint[1] * -0.001
                wpose_copy.position.z = waypoint[2] * 0.001
                new_rot = waypoint[3:6]
                new_quat = tf.transformations.quaternion_from_euler(*new_rot)
                wpose_copy.orientation.x, wpose_copy.orientation.y, wpose_copy.orientation.z, wpose_copy.orientation.w = new_quat
                waypoints_list.append(wpose_copy)
        else:
            print("mod를 지정하지 않았습니다. rel(상대),abs(절대) 중 지정해주세요.")
            return -1
        if Is_samepos(waypoints_list[0],self.move_group.get_current_pose().pose):
            # print("현위치를 웨이포인트로 지정한 부분을 스킵합니다.")
            waypoints_list.pop(0)
        return self.execute_cartesian_path(waypoints_list)

    def execute_cartesian_path(self, waypoints):
        if self.is_stopped:
            return
        max_attempts = 5
        for attempt in range(max_attempts):
            for eef_step in [0.001,0.002, 0.004, 0.008]:
                for jump_threshold in [0, 1.57]:
                    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, eef_step, jump_threshold, avoid_collisions=False)
                    if fraction == 1.0:
                        if attempt != 0:
                            print("{}회 시도로 계획됨, eef_step : {}, jump_threshold : {}".format(attempt + 1, eef_step, jump_threshold))
                        # self.display_trajectory(plan)
                        self.move_group.execute(plan, wait=True)
                        return
                    # elif fraction > 0.9:
                    #     self.display_trajectory(plan)
            rospy.sleep(0.05)
        print("경로 생성에 실패했습니다. 가능한 범위인지 확인하십시오.")
        self.move_group.clear_pose_targets()
    def display_trajectory(self,plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        for _ in range(2):
            self.display_trajectory_publisher.publish(display_trajectory)
            rospy.sleep(0.5)

    def stop(self):     # 현재 경로를 멈춤.
        self.move_group.stop()

class Gripper():
    def __init__(self,move_group):
        self.move_group = move_group
        if not isTest:
            self.gripper = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
            print("Gripper Connected.")
            self.gripper.write(b"\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
            data_raw = self.gripper.readline()
            print(data_raw)
            data = binascii.hexlify(data_raw)
            print("Response", data, '\n')
            time.sleep(0.01)
            self.gripper.write(b"\x09\x03\x07\xD0\x00\x01\x85\xCF")
            data_raw = self.gripper.readline()
            print(data_raw)
            data = binascii.hexlify(data_raw)
            print("Response", data)
            time.sleep(1)
            print("Gripper Initialized.")

    def open(self):
        if self.move_group.is_stopped:
            return
        if not isTest:
            self.gripper.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")

        print("Gripper Opened")
        time.sleep(0.5)

    def close(self):
        if self.move_group.is_stopped:
            return
        if not isTest:
            self.gripper.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")

        print("Gripper Closed")
        time.sleep(0.5)

def Pick(UR,gripper,num):
    if num == 1:
        waypoints0_1 = [posx_Table_high,posx_Obj1_high, posx_Obj1]
        UR.movel(waypoints0_1,"abs")
        gripper.close()
        UR.add_obstacle('box',[UR.adjusted_pose[0],UR.adjusted_pose[1],UR.adjusted_pose[2]-200, 220, 110, 20],rel=True)
        UR.scene.attach_object('box',link='tool0')
        waypoints1_0 = [posx_Obj1_high, posx_Table_high]
        UR.movel(waypoints1_0,"abs")
    if num == 2:
        waypoints0_2 = [posx_Table_high,posx_Obj2_high, posx_Obj2]
        UR.movel(waypoints0_2,"abs")
        gripper.close()
        UR.add_obstacle('box',[UR.adjusted_pose[0],UR.adjusted_pose[1],UR.adjusted_pose[2]-200, 220, 110, 20],rel=True)
        UR.scene.attach_object('box',link='tool0')
        waypoints2_0 = [posx_Obj2_high, posx_Table_high]
        UR.movel(waypoints2_0,"abs")
    if num == 3:
        waypoints0_3 = [posx_Table_high,posx_Obj3_high, posx_Obj3]
        UR.movel(waypoints0_3,"abs")
        gripper.close()
        UR.add_obstacle('box',[UR.adjusted_pose[0],UR.adjusted_pose[1],UR.adjusted_pose[2]-200, 220, 110, 20],rel=True)
        UR.scene.attach_object('box',link='tool0')
        waypoints3_0 = [posx_Obj3_high, posx_Table_high]
        UR.movel(waypoints3_0,"abs")

def Place(UR,gripper,num):
    if num == 1:
        waypoints0_1 = [posx_Table_high,posx_Obj1_high, posx_Obj1]
        UR.movel(waypoints0_1,"abs")
        gripper.open()
        UR.scene.remove_attached_object('tool0', name='box')
        waypoints1_0 = [posx_Obj1_high, posx_Table_high]
        UR.movel(waypoints1_0,"abs")
    if num == 2:
        waypoints0_2 = [posx_Table_high,posx_Obj2_high, posx_Obj2]
        UR.movel(waypoints0_2,"abs")
        gripper.open()
        UR.scene.remove_attached_object('tool0', name='box')
        waypoints2_0 = [posx_Obj2_high, posx_Table_high]
        UR.movel(waypoints2_0,"abs")
    if num == 3:
        waypoints0_3 = [posx_Table_high,posx_Obj3_high, posx_Obj3]
        UR.movel(waypoints0_3,"abs")
        gripper.open()
        UR.scene.remove_attached_object('tool0', name='box')
        waypoints3_0 = [posx_Obj3_high, posx_Table_high]
        UR.movel(waypoints3_0,"abs")

def Is_samepos(pose1, pose2, tolerance=0.0001):
    """
    두 포즈가 지정된 허용 오차 내에서 동일한지 확인합니다.
    
    :param pose1: 첫 번째 포즈 (geometry_msgs.msg.Pose)
    :param pose2: 두 번째 포즈 (geometry_msgs.msg.Pose)
    :param tolerance: 허용 오차
    :return: 두 포즈가 허용 오차 내에서 동일하면 True, 그렇지 않으면 False
    """
    position_close = (abs(pose1.position.x - pose2.position.x) < tolerance and
                      abs(pose1.position.y - pose2.position.y) < tolerance and
                      abs(pose1.position.z - pose2.position.z) < tolerance)
    
    orientation_close = (abs(pose1.orientation.x - pose2.orientation.x) < tolerance and
                         abs(pose1.orientation.y - pose2.orientation.y) < tolerance and
                         abs(pose1.orientation.z - pose2.orientation.z) < tolerance and
                         abs(pose1.orientation.w - pose2.orientation.w) < tolerance)
    
    return position_close and orientation_close

def signal_handler(sig, frame):
    global isInterrupted
    print("\nCtrl+C pressed. Stopping the robot...")
    isInterrupted = True
def main():
    global UR, isInterrupted
    signal.signal(signal.SIGINT, signal_handler)
    
    UR = Move_Group()
    gripper = Gripper(UR)
    try:
        UR.add_obstacle('Table',[-300, -400, -300, 300, 400, 0])
        UR.add_obstacle('workspace',[500, 350, -300, 900, -250, 200])
        
        actions = [
            (Pick, 1), (Place, 1),
            (Pick, 2), (Place, 2),
            (Pick, 3), (Place, 3)
        ]
        
        for action, num in actions:
            if isInterrupted:
                break
            action(UR, gripper, num)
        
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        shutdown()

def shutdown():
    global UR
    UR.scene.remove_world_object('box')        
    UR.stop()
    UR.is_stopped = True
    rospy.signal_shutdown("종료")
    moveit_commander.roscpp_shutdown()
    print("\033[1;33m종료됨.\033[0m")

if __name__ == "__main__":
    main()


