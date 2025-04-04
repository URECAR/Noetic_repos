''' 20240823. Ctrl+C 인터럽트 시 정지 구현, UR 펜던트서 정지 누를 시 다음의 일련 동작들 즉시 종료 ㅜ구현.
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



'''




###
# 0823 최적화됨.
###

'''
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
        self.move_group.set_max_velocity_scaling_factor(Max_Joint_VEL)
        self.move_group.set_max_acceleration_scaling_factor(Max_Joint_ACC)
        self.move_group.limit_max_cartesian_link_speed(Cartesian_Speed)
        rospy.wait_for_service('/ur_hardware_interface/dashboard/program_state')
        self.SRV_program_state = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
        self.program_state = None   # 펜던트의 프로그램 실행 상태
        print("\033[1;33m============")
        print("초기화 완료")
        print("============\033[0m")

        self.ui_thread = threading.Thread(target=self.update_current_state)
        self.ui_thread.daemon = True    # 메인 스레드 종료 시 같이 서브 스레드도 종료
        self.ui_thread.start()

        rospy.sleep(0.1)

    def update_current_state(self):
        rate = rospy.Rate(Rate)
        while not rospy.is_shutdown() and not isInterrupted:
            try:
                self.current_joints = [x * (360.0 / tau) for x in self.move_group.get_current_joint_values()]
                self.current_pose = self.move_group.get_current_pose().pose
                current_rot = tf.transformations.euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])
                self.adjusted_pose = [self.current_pose.position.x * -1000, self.current_pose.position.y * -1000, self.current_pose.position.z * 1000, current_rot[0], current_rot[1], current_rot[2]]

                if not isInterrupted:
                    program_state_response = self.SRV_program_state()
                    self.program_state = program_state_response.state.state
                    self.prog_stopped = (self.program_state == "STOPPED")
                    if self.prog_stopped:
                        print("\033[1;31mUR 제어에 실패함. \033[0;32m펜던트 내 프로그램 실행 여부를 확인해주십시오.\033[0m")
                        break

                if self.callback_count > 10:
                    self.callback_count -= 10
                self.callback_count += 1
            except rospy.ServiceException:
                if not isInterrupted:
                    rospy.logerr("서비스 호출 실패")
                break
            except rospy.ROSInterruptException:
                break
            rate.sleep()
        print("UR 업데이트 정지됨.")

    def add_obstacle(self, obj_name, coords, rel=False):
        if isInterrupted or self.prog_stopped:
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
        # rospy.loginfo(f"장애물 '{obj_name}' 생성 완료.")
        rospy.sleep(0.05)

    def movej(self, add_joints, Rel=False):
        if isInterrupted or self.prog_stopped:
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
        if isInterrupted or self.prog_stopped:
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
        if self.prog_stopped or self.prog_stopped:
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
        self.prog_stopped = True

class Gripper():
    def __init__(self):
        global UR
        self.UR = UR
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
        if isInterrupted or UR.prog_stopped:
            return
        if not isTest:
            self.gripper.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")

        print("Gripper Opened")
        time.sleep(0.5)

    def close(self):
        if isInterrupted or UR.prog_stopped:
            return
        if not isTest:
            self.gripper.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")

        print("Gripper Closed")
        time.sleep(0.5)

def Pick(num):
    global UR, gripper
    if isInterrupted or UR.prog_stopped:
        return
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
def Place(num):
    global UR, gripper
    if isInterrupted or UR.prog_stopped:
        return
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
    global UR, gripper, isInterrupted
    signal.signal(signal.SIGINT, signal_handler)
    
    UR = Move_Group()
    gripper = Gripper()
    try:
        UR.add_obstacle('Table',[-300, -400, -300, 300, 400, 0])
        UR.add_obstacle('workspace',[500, 350, -300, 900, -250, 200])
        Pick(1)
        Place(2)
        Pick(2)
        Place(3)
        Pick(3)
        Place(1)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        shutdown()

def shutdown():
    global UR
    UR.scene.remove_world_object('box')        
    UR.move_group.stop()
    rospy.signal_shutdown("종료")
    moveit_commander.roscpp_shutdown()
    print("\033[1;33m종료됨.\033[0m")

if __name__ == "__main__":
    main()

'''

'''
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
import traceback
group_name = "manipulator"
isTest = False
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
client_socket = None
server_socket = None
class Move_Group(object):
    def __init__(self,ignore_stop=False):
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
        self.prog_stopped = False
        self.ignore_stop = ignore_stop
        self.move_group.set_max_velocity_scaling_factor(Max_Joint_VEL)
        self.move_group.set_max_acceleration_scaling_factor(Max_Joint_ACC)
        self.move_group.limit_max_cartesian_link_speed(Cartesian_Speed)
        rospy.wait_for_service('/ur_hardware_interface/dashboard/program_state')
        self.SRV_program_state = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
        self.program_state = None   # 펜던트의 프로그램 실행 상태
        print("\033[1;33m============")
        print("초기화 완료")
        print("============\033[0m")

        self.ui_thread = threading.Thread(target=self.update_current_state)
        self.ui_thread.daemon = True    # 메인 스레드 종료 시 같이 서브 스레드도 종료
        self.ui_thread.start()

        rospy.sleep(0.1)

    def update_current_state(self):
        rate = rospy.Rate(Rate)
        while not rospy.is_shutdown() and not isInterrupted:
            try:
                self.current_joints = [x * (360.0 / tau) for x in self.move_group.get_current_joint_values()]
                self.current_pose = self.move_group.get_current_pose().pose
                current_rot = tf.transformations.euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])
                self.adjusted_pose = [self.current_pose.position.x * -1000, self.current_pose.position.y * -1000, self.current_pose.position.z * 1000, current_rot[0], current_rot[1], current_rot[2]]

                if not isInterrupted:
                    program_state_response = self.SRV_program_state()
                    self.program_state = program_state_response.state.state
                    if not self.ignore_stop:
                        self.prog_stopped = (self.program_state == "STOPPED")
                    if self.prog_stopped:
                        print("\033[1;31mUR 제어에 실패함. \033[0;32m펜던트 내 프로그램 실행 여부를 확인해주십시오.\033[0m")
                        break

                if self.callback_count > 10:

                    # print("\nCurrent Posx : [{:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}]".format(*self.adjusted_pose))
                    # print("Current Posj : [{:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}]".format(*self.current_joints))
                    # print(f"Program State: {self.program_state}")
                    self.callback_count -= 10
                self.callback_count += 1
            except rospy.ServiceException:
                if not isInterrupted:
                    rospy.logerr("서비스 호출 실패")
                break
            except rospy.ROSInterruptException:
                break
            rate.sleep()
        print("UR 업데이트 정지됨.")

    def add_obstacle(self, obj_name, coords, rel=False):
        if isInterrupted or self.prog_stopped:
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
        # rospy.loginfo(f"장애물 '{obj_name}' 생성 완료.")
        rospy.sleep(0.05)

    def movej(self, add_joints, Rel=False):
        if isInterrupted or self.prog_stopped:
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
        if isInterrupted or self.prog_stopped:
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
        if self.prog_stopped or self.prog_stopped:
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
        self.prog_stopped = True
        print("Move_Group stop method completed")

class Gripper():
    def __init__(self):
        global UR
        self.UR = UR
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
        if isInterrupted or UR.prog_stopped:
            return
        if not isTest:
            self.gripper.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")

        print("Gripper Opened")
        time.sleep(0.5)

    def close(self):
        if isInterrupted or UR.prog_stopped:
            return
        if not isTest:
            self.gripper.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")

        print("Gripper Closed")
        time.sleep(0.5)

def start_socket(server_ip, server_port, max_retries=5, retry_delay=5):
    global UR, gripper, client_socket, server_socket
    retries = 0
    while not isInterrupted and not UR.prog_stopped and retries < max_retries:
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((server_ip, server_port))
            server_socket.listen(5)
            print(f"서버가 {server_ip}:{server_port}에서 대기 중입니다.")
            client_socket, client_addr = server_socket.accept()
            print(f"{client_addr}에서 연결 수락됨")
            while not isInterrupted and not UR.prog_stopped:
                try:
                    command = client_socket.recv(1024).decode('utf-8')
                    if not command:
                        print("요청으로 인한 연결 종료 ")
                        break
                    print(f"클라이언트로부터 데이터 수신: {command}")
                    response = "received"
                    client_socket.send(response.encode('utf-8'))
                    if command == 'grip1':
                        Pick(1)
                    elif command == 'grip2':
                        Pick(2)
                    elif command == 'grip3':
                        Pick(3)
                    elif command == 'place1':
                        Place(1)
                    elif command == 'place2':
                        Place(2)
                    elif command == 'place3':
                        Place(3)
                    else:
                        print(f"잘못된 명령: {command}")
                    response = "complete"
                    if not isInterrupted:
                        client_socket.send(response.encode('utf-8'))
                except socket.timeout:
                    print("클라이언트 통신 타임아웃")
                except ConnectionResetError:
                    print("클라이언트 연결 리셋")
                except Exception as e:
                    if not isInterrupted:
                        print(f"클라이언트 통신 중 예외 발생: {str(e)}")
                        print(traceback.format_exc())
                    break
        except OSError as e:
            if not isInterrupted:
                print(f"서버 소켓 예외 발생: {e}")
                retries += 1
                if retries < max_retries:
                    print(f"{retry_delay}초 후 재시도합니다. (시도 {retries}/{max_retries})")
                    time.sleep(retry_delay)
                else:
                    print("최대 재시도 횟수를 초과했습니다. 프로그램을 종료합니다.")
                    break
        finally:
            if client_socket:
                client_socket.close()
            if server_socket:
                server_socket.close()
            print("소켓 종료")

def Pick(num):
    global UR, gripper
    if isInterrupted or UR.prog_stopped:
        return
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

def Place(num):
    global UR, gripper
    if isInterrupted or UR.prog_stopped:
        return
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
    shutdown()
def main():
    global UR, gripper, isInterrupted
    signal.signal(signal.SIGINT, signal_handler)
    
    UR = Move_Group()
    # UR = Move_Group(ignore_stop=True) # 기존엔 UR 로봇이 정지 중일 때 코드가 중단된다면, ignore_stop을 사용 시 로봇은 정지 중에도 코드가 작동함.
    gripper = Gripper()
    try:
        UR.add_obstacle('Table',[-300, -400, -300, 300, 400, 0])
        UR.add_obstacle('workspace',[500, 350, -300, 900, -250, 200])
        gripper.open()
        start_socket(Server_IP,Server_Port)
        # Pick(1)
        # Place(2)
        # Pick(2)
        # Place(3)
        # Pick(3)
        # Place(1)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if not isInterrupted:
            shutdown()
def shutdown():
    global UR, client_socket, server_socket
    UR.scene.remove_world_object('box')        
    UR.move_group.stop()
    rospy.signal_shutdown("종료")
    moveit_commander.roscpp_shutdown()
    print("\033[1;33m종료됨.\033[0m")
    if client_socket:
        client_socket.close()
    if server_socket:
        server_socket.close()

if __name__ == "__main__":
    main()

'''



'''
#완성본같음
#0827
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
import traceback
import pyrealsense2 as rs
import numpy as np
import cv2
from collections import defaultdict
group_name = "manipulator"          # Moveit이 MotionPlanning을 할 대상 그룹. 
isTest = False                      # Gripper가 장착되지 않은 Simulator를 위한 변수. True이면 그리퍼를 인식하지 않는다.


#### Base Variables ##############
Cartesian_Speed = 0.2               # Cartesian 이동의 속도 제한 [m/s]
Max_Joint_VEL   = 0.3               # Joint 이동의 속도 제한 [rad/s]
Max_Joint_ACC   = 0.3               # Joint 이동의 가속도 제한  [rad/s^2]
TABLE_EXT_HEIGHT = 500              # Object 위에 위치할 높이. 
Server_IP = '127.0.0.1'             # 127.0.0.1은 루프백이며 자신을 서버 주소로 함. 필요 시 원하는 서버 주소로 변경.
Server_Port = 5000
Rate = 10                           # State 확인 주기(hz)



#### Pose List ###
posj_Zero_pose = [0, -90, 90, -90, -90, 0]                      # 기본 자세. [x, y, z, rx, ry, rz]의 오일러 좌표계를 가짐. 각도는 degree.
posj_Detect_pose = [-5, -84.5, 71, -77.5, -90, 0]               # 물체 인식 자세. 최적의 인식 장소를 가져야 함. 수정 시 ROI 영역도 수정해야 함.
posx_Table_high = [-706.8, -55.80, 550.0, -3.14, 0.00, -1.58]   # 테이블 경유점

posx_Obj1 = [-707, -211, 416.5, -3.14, 0.00, -1.57]
posx_Obj2 = [-706, -64, 416.5, -3.14, 0.00, -1.58]
posx_Obj3 = [-706, 82, 416.5, -3.14, 0.00, -1.57]
posx_Obj1_high = posx_Obj1.copy()
posx_Obj1_high[2] = TABLE_EXT_HEIGHT
posx_Obj2_high = posx_Obj2.copy()
posx_Obj2_high[2] = TABLE_EXT_HEIGHT
posx_Obj3_high = posx_Obj3.copy()
posx_Obj3_high[2] = TABLE_EXT_HEIGHT






#### Init Variables ##############
isInterrupted = False           # Ctrl+C를 누르면 인터럽트 활성화.
UR = None                       # UR10e을 전역변수로 선언
realsense = None                # Realsense D435i 카메라를 전역변수로 선언
client_socket = None
server_socket = None
posx_Objexp_list = None



class Move_Group(object):   # 로봇팔의 움직임 제어 클래스
    def __init__(self, ignore_stop=False):
        super(Move_Group, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("move_group_ur", anonymous=True)
        self.robot = moveit_commander.RobotCommander()                          # moveit commander Init
        self.scene = moveit_commander.PlanningSceneInterface()                  # moveit world spawn
        self.move_group = moveit_commander.MoveGroupCommander(group_name)       # move_group instance Init
        # self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.planning_frame = self.move_group.get_planning_frame()
        self.current_joints = None
        self.current_pose = None
        self.adjusted_pose = None
        self.callback_count = 0
        self.prog_stopped = False
        self.ignore_stop = ignore_stop
        self.move_group.set_max_velocity_scaling_factor(Max_Joint_VEL)
        self.move_group.set_max_acceleration_scaling_factor(Max_Joint_ACC)
        self.move_group.limit_max_cartesian_link_speed(Cartesian_Speed)
        rospy.wait_for_service('/ur_hardware_interface/dashboard/program_state')
        self.SRV_program_state = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_state', GetProgramState)
        self.program_state = None   # 펜던트의 프로그램 실행 상태
        print("\033[1;33m============")
        print("초기화 완료")
        print("============\033[0m")

        self.ui_thread = threading.Thread(target=self.update_current_state)
        self.ui_thread.daemon = True    # 메인 스레드 종료 시 같이 서브 스레드도 종료
        self.ui_thread.start()

        rospy.sleep(0.1)

    def update_current_state(self):
        rate = rospy.Rate(Rate)
        while not rospy.is_shutdown() and not isInterrupted:
            try:
                self.current_joints = [x * (360.0 / tau) for x in self.move_group.get_current_joint_values()]
                self.current_pose = self.move_group.get_current_pose().pose
                current_rot = tf.transformations.euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])
                self.adjusted_pose = [self.current_pose.position.x * -1000, self.current_pose.position.y * -1000, self.current_pose.position.z * 1000, current_rot[0], current_rot[1], current_rot[2]]

                if not isInterrupted:
                    program_state_response = self.SRV_program_state()
                    self.program_state = program_state_response.state.state
                    if not self.ignore_stop:
                        self.prog_stopped = (self.program_state == "STOPPED")
                    if self.prog_stopped:
                        print("\033[1;31mUR 제어에 실패함. \033[0;32m펜던트 내 프로그램 실행 여부를 확인해주십시오.\033[0m")
                        break

                if self.callback_count > 10:

                    # print("\nCurrent Posx : [{:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}]".format(*self.adjusted_pose))
                    # print("Current Posj : [{:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}]".format(*self.current_joints))
                    # print(f"Program State: {self.program_state}")
                    self.callback_count -= 10
                self.callback_count += 1
            except rospy.ServiceException:
                if not isInterrupted:
                    rospy.logerr("State 업데이트 실패.")
                break
            except rospy.ROSInterruptException:
                break
            rate.sleep()
        print("UR 업데이트 정지됨.")

    def add_obstacle(self, obj_name, coords, rel=False):
        if isInterrupted or self.prog_stopped:
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
        # rospy.loginfo(f"장애물 '{obj_name}' 생성 완료.")
        rospy.sleep(0.05)

    def movej(self, add_joints, Rel=False):         # 조인트 공간 좌표 이동. 
        if isInterrupted or self.prog_stopped:      # 인터럽트 or 프로그램 정지 시 메서드 무시
            return
        if Rel:                                     # 상대 이동 시 조인트를 수치만큼 더 이동
            joint_goal = [current_joint + (tau / 360) * add_joint for current_joint, add_joint in zip(self.current_joints, add_joints)]
        else:                                       # 절대 이동 시 조인트를 수치만큼 이동
            joint_goal = [(tau / 360) * add_joint for add_joint in add_joints]

        success = self.move_group.go(joint_goal, wait=True)
        if not success:
            print("Normal Move Failed!")
        self.move_group.stop()

    def movel(self, waypoints, mod="None"):         # 직교 공간 좌표 이동. 웨이포인트 다수 지정 가능.
        if isInterrupted or self.prog_stopped:      
            return
        waypoints_list = []
        wpose = self.move_group.get_current_pose().pose
        if mod == "rel":
            for waypoint in waypoints:
                wpose_copy = copy.deepcopy(wpose)
                wpose_copy.position.x += waypoint[0] * -0.001  # 밀리미터 단위로 이동. (UR로봇과 x,y는 좌표계가 반대임.)
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
        if self.Is_samepos(waypoints_list[0],self.move_group.get_current_pose().pose):   # 현위치를 웨이포인트로 잡으면 경로 설정이 안됨.
            waypoints_list.pop(0)
        return self.execute_cartesian_path(waypoints_list)

    def Is_samepos(self,pose1, pose2, tolerance=0.0001):
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

    def execute_cartesian_path(self, waypoints):
        if self.prog_stopped or self.prog_stopped:
            return
        max_attempts = 5
        for attempt in range(max_attempts):
            for eef_step in [0.001,0.002, 0.004, 0.008]:        # eef_step : 경로 지정 분할 간격. 0.001이면 1mm마다 재계산하여 직선 경로를 만들어냄.
                for jump_threshold in [0, 1.57]:                # jump_threshold : 경로 생성 시  Jump의 한계치 지정. 조인트 값이 튀는 정도를 조정.
                    (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, eef_step, jump_threshold, avoid_collisions=False)
                    if fraction == 1.0:     # 경로가 100% 구성됐을 시
                        if attempt != 0:    # 단번에 계획 안 됐을 시 출력. 
                            print("{}회 시도로 계획됨, eef_step : {}, jump_threshold : {}".format(attempt + 1, eef_step, jump_threshold))
                        # self.display_trajectory(plan)
                        self.move_group.execute(plan, wait=True)
                        return
            rospy.sleep(0.05)
        print("경로 생성에 실패했습니다. 가능한 범위인지 확인하십시오.")
        self.move_group.clear_pose_targets()

    def stop(self):     # 현재 경로를 멈춤.
        self.move_group.stop()
        self.prog_stopped = True
        print("Move_Group stop method completed")

class Gripper():            # 로봇팔 말단(eef)의 그리퍼 제어 클래스
    def __init__(self):
        global UR
        self.UR = UR
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
        if isInterrupted or UR.prog_stopped:
            return
        if not isTest:
            self.gripper.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")

        print("Gripper Opened")
        time.sleep(0.5)

    def close(self):
        if isInterrupted or UR.prog_stopped:
            return
        if not isTest:
            self.gripper.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")

        print("Gripper Closed")
        time.sleep(0.5)

class RealSense:            # 로봇팔 말단(eef)에 부착된 Realsense D435i 제어 클래스
    def __init__(self, duration=2):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.coordinate_store = defaultdict(int)
        self.ROI_POINTS = np.array([(69, 224), (565, 227), (562, 479), (64, 477)], np.int32)
        self.EDGE_THRESHOLD_LOW = 40
        self.EDGE_THRESHOLD_HIGH = 150
        self.MIN_CONTOUR_AREA = 5000
        self.MIN_RECT_LENGTH = 50
        self.PIXEL_TOLERANCE = 20
        self.DETECTION_THRESHOLD = 0.4
        self.duration = duration
        self.coordinate_store = defaultdict(int)
        self.pixel_points = np.float32([[463, 378], [321, 370], [178, 367]])
        self.ur_points = np.float32([[-946.5, -208.6], [-941, -69.4], [-939.9, 71]])
        self.transform_matrix = cv2.getAffineTransform(self.pixel_points, self.ur_points)
        self.coord_offset = [5,0]   # UR 좌표계로 변환 시 offset
    def pixel_to_ur(self, pixel_coord):
        # 픽셀 좌표를 UR 좌표로 변환
        ur_coord = cv2.transform(np.array([[pixel_coord]]), self.transform_matrix)[0][0]
        ur_coord[0] -= self.coord_offset[0]     # UR 좌표계는 부호가 반대임.
        ur_coord[1] -= self.coord_offset[1] 
        return ur_coord
    
    def process_frame(self, color_image):
        height, width = color_image.shape[:2]

        # ROI 마스크 생성
        mask = np.zeros(color_image.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [self.ROI_POINTS], 255)              # ROI 영역 정의

        # ROI 적용
        roi = cv2.bitwise_and(color_image, color_image, mask=mask)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 엣지 검출 파라미터 조정
        edges = cv2.Canny(blurred, 30, 100)

        # 모폴로지 연산 추가
        kernel = np.ones((5,5), np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=1)
        edges = cv2.erode(edges, kernel, iterations=1)

        # 윤곽선 검출
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        rectangles = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.MIN_CONTOUR_AREA:
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                if len(approx) == 4:  # 사각형인 경우만 고려
                    rect = cv2.minAreaRect(approx)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)

                    width = min(rect[1])
                    height = max(rect[1])

                    if min(width, height) >= self.MIN_RECT_LENGTH:
                        rectangles.append((box, rect))

        # 중복 제거 및 가장 큰 직사각형 선택
        filtered_rectangles = []
        for i, (box1, rect1) in enumerate(rectangles):
            is_duplicate = False
            for j, (box2, rect2) in enumerate(rectangles):
                if i != j:
                    center1 = rect1[0]
                    center2 = rect2[0]
                    distance = np.sqrt((center1[0] - center2[0])**2 + (center1[1] - center2[1])**2)
                    if distance < 50:  # 중심점 거리가 50픽셀 이내인 경우 중복으로 간주
                        is_duplicate = True
                        if cv2.contourArea(box1) > cv2.contourArea(box2):
                            rectangles[j] = (box1, rect1)
                        break
            if not is_duplicate:
                filtered_rectangles.append((box1, rect1))

        detected_centers = []
        for box, rect in filtered_rectangles:
            cv2.drawContours(color_image, [box], 0, (0, 0, 255), 2)

            center = tuple(map(int, rect[0]))
            ur_coord = self.pixel_to_ur(center)

            angle = rect[2]
            width, height = rect[1]

            # 짧은 변의 방향으로 화살표가 향하도록 수정
            if width < height:
                angle = angle - 180 if angle > 0 else angle + 180
            else:
                angle = angle - 90 if angle > 0 else angle + 90

            angle_rad = math.radians(angle)

            detected_centers.append((center, ur_coord, angle_rad))

            direction = (int(center[0] + 50 * math.cos(math.radians(angle))),
                         int(center[1] + 50 * math.sin(math.radians(angle))))
            cv2.arrowedLine(color_image, center, direction, (0, 255, 0), 2)

            # 변환된 UR 좌표를 이미지에 표시
            cv2.putText(color_image, f"UR: ({ur_coord[0]:.0f}, {ur_coord[1]:.0f})", 
                        (center[0]+10, center[1]+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.polylines(color_image, [self.ROI_POINTS], True, (255, 255, 0), 2)

        return color_image, edges, detected_centers

    def update_coordinate_store(self, detected_centers):
        for center, ur_coord, angle_rad in detected_centers:
            x, y = center
            matched = False
            for (cx, cy), (count, _, _) in list(self.coordinate_store.items()):
                if abs(cx - x) <= self.PIXEL_TOLERANCE and abs(cy - y) <= self.PIXEL_TOLERANCE:
                    new_cx = (cx * count + x) / (count + 1)
                    new_cy = (cy * count + y) / (count + 1)
                    new_ur_x = (self.coordinate_store[(cx, cy)][1][0] * count + ur_coord[0]) / (count + 1)
                    new_ur_y = (self.coordinate_store[(cx, cy)][1][1] * count + ur_coord[1]) / (count + 1)
                    new_angle = (self.coordinate_store[(cx, cy)][2] * count + angle_rad) / (count + 1)
                    del self.coordinate_store[(cx, cy)]
                    self.coordinate_store[(int(new_cx), int(new_cy))] = (count + 1, (new_ur_x, new_ur_y), new_angle)
                    matched = True
                    break
            if not matched:
                self.coordinate_store[(x, y)] = (1, ur_coord, angle_rad)

    def get_stable_coordinates(self,frame_count):
        stable_coordinates = []
        for (cx, cy), (count, ur_coord, angle_rad) in self.coordinate_store.items():
            # if count >= self.DETECTION_THRESHOLD:
            if count >= frame_count * self.DETECTION_THRESHOLD:
                stable_coordinates.append((cx, cy, count, ur_coord, angle_rad))
        return stable_coordinates

    def show_camera_feed(self):
        self.coordinate_store = defaultdict(int)
        self.pipeline.start(self.config)
        frame_count = 0  # 프레임 수를 계산하기 위한 변수

        try:
            start_time = time.time()
            while time.time() - start_time < self.duration:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                processed_image, edges, detected_centers = self.process_frame(color_image)

                self.update_coordinate_store(detected_centers)

                # 두 개의 이미지를 하나의 창으로 통합
                combined_image = np.hstack((processed_image, cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)))

                cv2.imshow('Processed and Edges', combined_image)
                frame_count += 1  # 각 프레임마다 증가

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            stable_coordinates = self.get_stable_coordinates(frame_count)
            
            if stable_coordinates:
                # x 좌표를 기준으로 내림차순 정렬
                sorted_coordinates = sorted(stable_coordinates, key=lambda x: x[0], reverse=True)
                result = {}
                for i, coord in enumerate(sorted_coordinates, 1):
                    pixel_coord = (coord[0], coord[1])
                    ur_coord = coord[3]
                    angle_rad = coord[4]
                    result[f'{i}'] = {
                        'pixel_coord': pixel_coord,
                        'ur_coord': ur_coord,
                        'angle_rad': angle_rad
                    }
                    print(f"Obj {i}: 픽셀 좌표: {pixel_coord}, UR 로봇 좌표: ({ur_coord[0]:.0f}, {ur_coord[1]:.0f}, _), 비틀림각: {angle_rad:.2f} rad, 인식률: {coord[2]/frame_count * 100.0:.2f}%")
            else:
                rospy.logerr("물체 인식 실패")
                result = {}
        except Exception as e:
            print(e)
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()
            return result

def start_socket(server_ip, server_port, max_retries=5, retry_delay=5): # 네트워크 소켓 통신
    global UR, gripper, client_socket, server_socket,posx_Objexp_list
    retries = 0
    while not isInterrupted and not UR.prog_stopped and retries < max_retries:
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((server_ip, server_port))
            server_socket.listen(5)
            print(f"서버가 {server_ip}:{server_port}에서 대기 중입니다.")
            client_socket, client_addr = server_socket.accept()
            print(f"{client_addr}에서 연결 수락됨")
            while not isInterrupted and not UR.prog_stopped:
                try:
                    command = client_socket.recv(1024).decode('utf-8')
                    if not command:
                        print("요청으로 인한 연결 종료 ")
                        break
                    print(f"클라이언트로부터 데이터 수신: {command}")
                    response = "received"
                    client_socket.send(response.encode('utf-8'))
                    if command == 'gripinner1':
                        UR.movej(posj_Detect_pose)
                        Pick("inner",1)
                    elif command == 'gripinner2':
                        Pick("inner",2)
                    elif command == 'gripinner3':
                        Pick("inner",3)
                    elif command == 'gripouter1':
                        Pick("outer",1)
                    elif command == 'gripouter2':
                        Pick("outer",2)
                    elif command == 'gripouter3':
                        Pick("outer",3)
                    elif command == 'place1':
                        Place(1)
                    elif command == 'place2':
                        Place(2)
                    elif command == 'place3':
                        Place(3)
                    elif command == 'Cam':
                        UR.movej(posj_Detect_pose)
                        posx_Objexp_list = realsense.show_camera_feed()                
                    elif command == "TEST":
                        try:
                            object_count = realsense.depth_counting()
                            print(f"물체 갯수: {object_count}")
                        except Exception as e:
                            print(f"Depth counting error: {e}")
                    elif command == "TEST2":
                        pass
                    else:
                        print(f"잘못된 명령: {command}")
                    response = "complete"
                    if not isInterrupted:
                        client_socket.send(response.encode('utf-8'))
                except socket.timeout:
                    print("클라이언트 통신 타임아웃")
                except ConnectionResetError:
                    print("클라이언트 연결 리셋")
                except Exception as e:
                    if not isInterrupted:
                        print(f"클라이언트 통신 중 예외 발생: {str(e)}")
                        print(traceback.format_exc())
                    break
        except OSError as e:
            if not isInterrupted:
                print(f"서버 소켓 예외 발생: {e}")
                retries += 1
                if retries < max_retries:
                    print(f"{retry_delay}초 후 재시도합니다. (시도 {retries}/{max_retries})")
                    time.sleep(retry_delay)
                else:
                    print("최대 재시도 횟수를 초과했습니다. 프로그램을 종료합니다.")
                    break
        finally:
            if client_socket:
                client_socket.close()
            if server_socket:
                server_socket.close()
            print("소켓 종료")

def Pick(option, num):
    global UR, gripper, posx_Objexp_list
    if isInterrupted or UR.prog_stopped:
        return

    if option == "inner":
        posx_Obj = [posx_Obj1, posx_Obj2, posx_Obj3][num - 1]
        posx_Obj_high = [posx_Obj1_high, posx_Obj2_high, posx_Obj3_high][num - 1]
    elif option == "outer":
        ur_coord = posx_Objexp_list[str(num)]['ur_coord']
        angle_rad = - posx_Objexp_list[str(num)]['angle_rad']
        if angle_rad > 1.57:
            actual_angle = angle_rad - 4.71
        elif angle_rad < -1.57:
            actual_angle = angle_rad + 1.57
        else:
            actual_angle = angle_rad - 1.57
        posx_Obj = [ur_coord[0], ur_coord[1], 408.5, -3.14, 0, actual_angle]
        posx_Obj_high = posx_Obj.copy()
        posx_Obj_high[2] = TABLE_EXT_HEIGHT
    else:
        print("Invalid option. Use 'inner' or 'outer'.")
        return

    waypoints = [posx_Table_high, posx_Obj_high, posx_Obj]
    UR.movel(waypoints, "abs")
    gripper.close()
    UR.add_obstacle('box', [UR.adjusted_pose[0], UR.adjusted_pose[1], UR.adjusted_pose[2]-200, 220, 110, 20], rel=True)
    UR.scene.attach_object('box', link='tool0')
    waypoints_return = [posx_Obj_high, posx_Table_high]
    UR.movel(waypoints_return, "abs")

    if option == "outer":
        print(f"Picked object at: {posx_Obj}")

def Place(num):
    global UR, gripper
    if isInterrupted or UR.prog_stopped:
        return
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

def signal_handler(sig, frame): # Ctrl+C 인터럽트 시 실행하여 안전 종료.
    global isInterrupted
    if not isInterrupted:
        print("\nCtrl+C pressed. Stopping the robot...")
        isInterrupted = True
        shutdown()

def main():                     # 메인 함수
    global UR, gripper, isInterrupted, realsense
    signal.signal(signal.SIGINT, signal_handler)
    realsense = RealSense(duration=2)
    UR = Move_Group()
    # UR = Move_Group(ignore_stop=True) # 기존엔 UR 로봇이 정지 중일 때 코드가 중단된다면, ignore_stop을 사용 시 로봇은 정지 중에도 코드가 작동함.
    UR.movej(posj_Detect_pose)
    gripper = Gripper()
    try:
        UR.add_obstacle('Table',[-400, -300, -300, 400, 300, 0])
        UR.add_obstacle('Workspace',[500, 350, -300, 1100, -250, 200])
        UR.add_obstacle('Machine',[1000, -600, -300, -700, -2500, 1200])
        gripper.open()
        start_socket(Server_IP,Server_Port)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if not isInterrupted:
            shutdown()

def shutdown():                 # 안전종료 메서드
    global UR, client_socket, server_socket
    UR.scene.remove_world_object('box')        
    UR.move_group.stop()
    rospy.signal_shutdown("종료")
    moveit_commander.roscpp_shutdown()
    print("\033[1;33m종료됨.\033[0m")
    if client_socket:
        client_socket.close()
    if server_socket:
        server_socket.close()

if __name__ == "__main__":
    main()


'''
