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
from collections import defaultdict, Counter

group_name = "manipulator"          # Moveit이 MotionPlanning을 할 대상 그룹. 
isTest = False                      # Gripper가 장착되지 않은 Simulator를 위한 변수. True이면 그리퍼를 인식하지 않는다.


#### Base Variables ##############
Cartesian_Speed = 0.6               # Cartesian 이동의 속도 제한 [m/s]
Max_Joint_VEL   = 0.3               # Joint 이동의 속도 제한 [rad/s]
Max_Joint_ACC   = 0.2               # Joint 이동의 가속도 제한  [rad/s^2]
TABLE_EXT_HEIGHT = 515              # Object 위에 위치할 높이. [mm] 
Server_IP = '127.0.0.1'             # 127.0.0.1은 루프백이며 자신을 서버 주소로 함. 필요 시 원하는 서버 주소로 변경.
Server_Port = 5000
Rate = 10                           # State 확인 주기(hz)
Gripper_port = '/dev/ttyUSB0'


#### Pose List ###
posj_Zero_pose = [0, -90, 90, -90, -90, 0]                      # 기본 자세. [x, y, z, rx, ry, rz]의 오일러 좌표계를 가짐. 각도는 degree.
posj_Detect_pose = [-15.62, -64.17, 44.62, -71.93, -90.02, -8.14]               # 물체 인식 자세. 최적의 인식 장소를 가져야 함. 수정 시 ROI 영역도 수정해야 함.
posj_Neutral_pose = [-27,-102, 103, -91, -90,-26]
posx_Table_high = [-1089.8, 100.80, 650.0, -3.14, 0.00, -1.58]   # 테이블 경유점

posx_Obj1 = [-871.43, 5.21, 420.0, 3.14, 0.00, -1.61]
posx_Obj2 = [-869.11, 150.71, 420.0, 3.14, 0.00, -1.61]
posx_Obj3 = [-866.88, 300.22, 420.0, -3.14, 0.00, -1.61]
posx_Obj1_high = posx_Obj1.copy()
posx_Obj1_high[2] = TABLE_EXT_HEIGHT
posx_Obj2_high = posx_Obj2.copy()
posx_Obj2_high[2] = TABLE_EXT_HEIGHT
posx_Obj3_high = posx_Obj3.copy()
posx_Obj3_high[2] = TABLE_EXT_HEIGHT

posx_Mid = [-353.78, 258.4, 574.6, -3.14, 0.00, -1.61]

posx_Machine = [115.1, 992.88, 488.45, -3.14, -0.00, -1.61]
posx_Machine_high = posx_Machine.copy()
posx_Machine_high[2] = 533




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
        self.current_joints = None
        self.current_pose = None    # quaternion 사용하는 ur pos
        self.callback_count = 0     # 콜백
        self.prog_stopped = False
        self.ignore_stop = ignore_stop
        self.move_group.set_max_velocity_scaling_factor(Max_Joint_VEL)
        self.move_group.set_max_acceleration_scaling_factor(Max_Joint_ACC)
        self.move_group.limit_max_cartesian_link_speed(Cartesian_Speed)
        self.move_group.set_end_effector_link("tool0")  # "tool0"을 실제 로봇의 엔드 이펙터 링크 이름으로 변경
        # PREEMPTED 상태 감지를 위한 변수 추가 
        self.preempted_detected = False        
        # ROS 로그 콜백 설정
        rospy.Subscriber("/rosout", rosgraph_msgs.msg.Log, self._log_callback)


        print("\033[1;33m============")
        print("초기화 완료")
        print("============\033[0m")

        self.ui_thread = threading.Thread(target=self.update_current_state)
        self.ui_thread.daemon = True    # 메인 스레드 종료 시 같이 서브 스레드도 종료
        self.ui_thread.start()

        rospy.sleep(0.1)

    def _log_callback(self, data):
        """ROS 로그 메시지를 모니터링하여 PREEMPTED 감지"""
        try:
            if "PREEMPTED" in data.msg:
                rospy.logwarn("PREEMPTED 상태 감지됨")
                self.preempted_detected = True
        except Exception as e:
            pass

    def update_current_state(self):
        rate = rospy.Rate(Rate)
        while not rospy.is_shutdown() and not isInterrupted:
            try:
                # wpose = self.move_group.get_current_pose().pose
                self.current_joints = [x * (360.0 / tau) for x in self.move_group.get_current_joint_values()]
                if self.callback_count > 10:
                    # print("Current Posj : [{:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}]".format(*self.current_joints))
                    # print(f"Program State: {self.program_state}")
                    self.callback_count -= 10
                self.callback_count += 1

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
        self.current_joints = [x * (360.0 / tau) for x in self.move_group.get_current_joint_values()]
        if isInterrupted or self.prog_stopped:      # 인터럽트 or 프로그램 정지 시 메서드 무시
            return
        if Rel:                                     # 상대 이동 시 조인트를 수치만큼 더 이동
            joint_goal = [current_joint + (tau / 360) * add_joint for current_joint, add_joint in zip(self.current_joints, add_joints)]
        else:                                       # 절대 이동 시 조인트를 수치만큼 이동
            joint_goal = [(tau / 360) * add_joint for add_joint in add_joints]

        success = self.move_group.go(joint_goal, wait=True)
        if not success:
            print(" MoveJ Failed!")
        self.move_group.stop()

    def movel(self, waypoints, mod="None"):         # 직교 공간 좌표 이동. 웨이포인트 다수 지정 가능.
        if isInterrupted or self.prog_stopped:      
            return False
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
            return False
            
        # 웨이포인트 필터링 향상: 현재 위치와 비슷한 첫 번째 웨이포인트 제거
        if waypoints_list and self.Is_samepos(waypoints_list[0], self.move_group.get_current_pose().pose, tolerance=0.001):
            # print("현재 위치와 첫 웨이포인트가 너무 가까워 제거합니다.")
            waypoints_list.pop(0)
            
        # 웨이포인트가 없으면 바로 종료
        if not waypoints_list:
            # print("이동할 웨이포인트가 없습니다.")
            return True  # 이미 목표 위치에 있으므로 성공으로 처리

        self.preempted_detected = False
        
        # 실행
        result = self.execute_cartesian_path(waypoints_list)
        
        # 실행 후 PREEMPTED 상태 확인
        if self.preempted_detected:
            rospy.logwarn("PREEMPTED 상태가 감지되어 1초 후 재시도합니다.")
            rospy.sleep(1.0)
            return self.movel(waypoints, mod)
        
        return result

    def Is_samepos(self, pose1, pose2, tolerance=0.001):
        """
        두 포즈가 지정된 허용 오차 내에서 동일한지 확인합니다.
        
        :param pose1: 첫 번째 포즈 (geometry_msgs.msg.Pose)
        :param pose2: 두 번째 포즈 (geometry_msgs.msg.Pose)
        :param tolerance: 허용 오차 (미터 단위)
        :return: 두 포즈가 허용 오차 내에서 동일하면 True, 그렇지 않으면 False
        """
        # 위치 차이 계산
        position_diff = math.sqrt(
            (pose1.position.x - pose2.position.x) ** 2 +
            (pose1.position.y - pose2.position.y) ** 2 +
            (pose1.position.z - pose2.position.z) ** 2
        )
        
        # 방향 차이 계산 (쿼터니언 내적 사용)
        orientation_diff = 1.0 - (
            pose1.orientation.x * pose2.orientation.x +
            pose1.orientation.y * pose2.orientation.y +
            pose1.orientation.z * pose2.orientation.z +
            pose1.orientation.w * pose2.orientation.w
        ) ** 2
        
        # 위치와 방향이 모두 허용 오차 내에 있는지 확인
        return position_diff < tolerance and orientation_diff < tolerance * 0.1
    def execute_cartesian_path(self, waypoints):
        if self.prog_stopped:
            return
        max_attempts = 5
        
        # 속도에 따른 eef_step 조정 (더 느린 속도일 경우 더 작은 스텝으로)
        base_eef_step = 0.004  # 기본 스텝 크기
        scaled_step = base_eef_step * max(0.2, Cartesian_Speed / 0.4)  # 최소 스텝 크기 보장
        
        for attempt in range(max_attempts):
            for eef_step in [scaled_step, scaled_step*2]:  # 조정된 스텝 크기 사용
                for jump_threshold in [0, 1.57]:  # 두 가지 점프 임계값 시도
                    (plan, fraction) = self.move_group.compute_cartesian_path(
                        waypoints, 
                        eef_step, 
                        jump_threshold
                        # avoid_collisions=True  # 충돌 회피 사용
                    )
                    if fraction == 1.0:  # 100% 경로 생성 성공
                        if attempt != 0:  # 단번에 계획 안 됐을 시 출력
                            print(f"카테시안 경로 계획 성공: {attempt+1}회 시도, eef_step: {eef_step}, jump_threshold: {jump_threshold}")
                        # self.display_trajectory(plan)
                        try:
                            self.move_group.execute(plan, wait=True)
                        except Exception as e:
                            if "PREEMPTED" in str(e):
                                execute_cartesian_path(waypoints)
                                pass
                        return True
                rospy.sleep(0.05)  # 다음 시도 전 잠시 대기
            
        print("카테시안 경로 생성 실패: 목표 위치가 도달 가능한지 확인하세요.")
        self.move_group.clear_pose_targets()
        return False

    def stop(self):     # 현재 경로를 멈춤.
        self.move_group.stop()
        self.prog_stopped = True
        print("Move_Group stop method completed")

class Gripper():            # 로봇팔 말단(eef)의 그리퍼 제어 클래스
    def __init__(self):
        global UR
        self.UR = UR
        if not isTest:
            self.gripper = serial.Serial(port=Gripper_port, baudrate=115200, timeout=1, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
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
        time.sleep(0.7)

    def close(self):
        if isInterrupted or UR.prog_stopped:
            return
        if not isTest:
            self.gripper.write(b"\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")

        print("Gripper Closed")
        time.sleep(0.7)

class RealSense:            # 로봇팔 말단(eef)에 부착된 Realsense D435i 제어 클래스
    def __init__(self, duration=2):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.coordinate_store = defaultdict(int)
        self.ROI_POINTS = np.array([(61, 227), (547, 174), (578, 432), (82, 475)], np.int32)    # 인식 범위를 픽셀 꼭짓점으로 지정
        self.MIN_CONTOUR_AREA = 10000    # 최소 인식 직사각형 넓이
        self.MAX_CONTOUR_AREA = 20000
        self.MAX_RECT_LENGTH = 300
        self.MIN_RECT_LENGTH = 50       # '' 길이
        self.PIXEL_TOLERANCE = 30
        self.DETECTION_THRESHOLD = 0.1  # 인식률. 
        self.duration = duration
        self.coordinate_store = defaultdict(int)
        self.pixel_points = np.float32([[447, 301], [314, 325], [175, 345]])
        self.ur_points = np.float32([[-1084.08, 18.88], [-1092.03, 150.67], [-1096.20, 287.8]])
        self.transform_matrix = cv2.getAffineTransform(self.pixel_points, self.ur_points)
        self.coord_offset = [3,0]   # UR 좌표계로 변환 시 offset
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
        cv2.fillPoly(mask, [self.ROI_POINTS], 255)
        
        # ROI 내부 영역을 약간 축소한 마스크 생성 (경계로부터의 여백 설정)
        margin = 3  # ROI 경계로부터의 여백 픽셀 수
        eroded_mask = cv2.erode(mask, np.ones((margin, margin), np.uint8), iterations=1)
        
        # ROI 적용
        roi = cv2.bitwise_and(color_image, color_image, mask=mask)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (0, 0),10)
        sharpened = cv2.addWeighted(gray, 13, blurred, -10, 0)

        # 엣지 검출 파라미터 조정
        edges = cv2.Canny(sharpened, 80, 150)
        
        # 축소된 마스크를 엣지에 적용하여 ROI 경계 근처의 엣지 제거
        edges = cv2.bitwise_and(edges, edges, mask=eroded_mask)

        # 모폴로지 연산
        kernel = np.ones((3,3), np.uint8)  # 작은 직사각형 보존을 위해 커널 크기 축소
        edges = cv2.dilate(edges, kernel, iterations=1)
        edges = cv2.erode(edges, kernel, iterations=1)

        # 윤곽선 검출 - 내부 윤곽선도 포함하도록 RETR_LIST 사용
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        rectangles = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.MAX_CONTOUR_AREA > area > self.MIN_CONTOUR_AREA:
                # 윤곽선의 중심점 계산
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    # 윤곽선의 중심점이 축소된 마스크 내부에 있는지 확인
                    if eroded_mask[cY, cX] == 255:  # 마스크 내부에 있음
                        epsilon = 0.02 * cv2.arcLength(contour, True)
                        approx = cv2.approxPolyDP(contour, epsilon, True)
                        
                        if len(approx) == 4:  # 사각형인 경우만 고려
                            rect = cv2.minAreaRect(approx)
                            box = cv2.boxPoints(rect)
                            box = np.int0(box)

                            width = min(rect[1])
                            height = max(rect[1])

                            if self.MAX_RECT_LENGTH >= min(width, height) >= self.MIN_RECT_LENGTH:
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

        # 결과를 시각화
        detected_centers = []
        for box, rect in filtered_rectangles:
            cv2.drawContours(color_image, [box], 0, (0, 0, 255), 2)

            center = tuple(map(int, rect[0]))
            ur_coord = self.pixel_to_ur(center)
            angle_offset = 7
            angle = rect[2] + angle_offset
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

        # 원본 ROI 경계와 축소된 ROI 경계 표시 (디버깅용)
        cv2.polylines(color_image, [self.ROI_POINTS], True, (255, 255, 0), 2)  # 원본 ROI
        
        # 축소된 ROI 경계 표시를 위한 윤곽선 찾기 (옵션)
        eroded_contours, _ = cv2.findContours(eroded_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(color_image, eroded_contours, -1, (0, 255, 255), 2)  # 축소된 ROI

        return color_image, edges, detected_centers

    def update_coordinate_store(self, detected_centers):
        """
        검출된 중심점들을 추적하고 업데이트하는 메서드입니다.
        새로 검출된 중심점이 기존에 저장된 중심점과 가까우면 평균을 내어 위치를 업데이트하고,
        그렇지 않으면 새로운 중심점으로 추가합니다.
        
        Args:
            detected_centers: 검출된 중심점 목록. 각 항목은 (center, ur_coord, angle_rad) 형식.
                - center: 픽셀 좌표 (x, y)
                - ur_coord: UR 로봇 좌표 (x, y)
                - angle_rad: 비틀림 각도 (라디안)
        """
        for center, ur_coord, angle_rad in detected_centers:
            # 검출된 중심점의 픽셀 좌표
            x, y = center
            matched = False
            
            # 기존에 저장된 모든 중심점을 순회
            for (cx, cy), (count, _, _) in list(self.coordinate_store.items()):
                # 기존 중심점과 현재 중심점 사이의 거리가 허용 오차(PIXEL_TOLERANCE) 이내인 경우
                if abs(cx - x) <= self.PIXEL_TOLERANCE and abs(cy - y) <= self.PIXEL_TOLERANCE:
                    # 새로운 중심점 좌표 계산 (가중 평균)
                    new_cx = (cx * count + x) / (count + 1)
                    new_cy = (cy * count + y) / (count + 1)
                    
                    # 새로운 UR 로봇 좌표 계산 (가중 평균)
                    new_ur_x = (self.coordinate_store[(cx, cy)][1][0] * count + ur_coord[0]) / (count + 1)
                    new_ur_y = (self.coordinate_store[(cx, cy)][1][1] * count + ur_coord[1]) / (count + 1)
                    
                    # 새로운 비틀림 각도 계산 (가중 평균)
                    new_angle = (self.coordinate_store[(cx, cy)][2] * count + angle_rad) / (count + 1)
                    
                    # 기존 항목 삭제
                    del self.coordinate_store[(cx, cy)]
                    
                    # 업데이트된 정보로 다시 저장
                    # (카운트+1, (UR X좌표, UR Y좌표), 비틀림 각도)
                    self.coordinate_store[(int(new_cx), int(new_cy))] = (count + 1, (new_ur_x, new_ur_y), new_angle)
                    matched = True
                    break
                    
            # 기존 중심점과 일치하는 것이 없으면 새로운 항목으로 추가
            if not matched:
                # (카운트=1, (UR X좌표, UR Y좌표), 비틀림 각도)
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
                # y 좌표를 기준으로 내림차순 정렬
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

    def calibrate(self):
        """
        레이저 포인터의 최대 y 좌표를 찾아 클러스터링 후 중앙값을 계산합니다.
        """
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # 최소 및 최대 x, y 값 설정
        min_x_threshold = 250
        max_x_threshold = 290
        min_y_threshold = 200
        max_y_threshold = 300
        
        # y좌표 기록을 위한 리스트 초기화
        y_coordinates = []
        
        # 파이프라인 시작
        pipeline.start(config)
        
        try:
            # 2초 동안 프레임을 수집하고 분석
            start_time = time.time()
            while time.time() - start_time < 2:
                # 프레임 얻기
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                
                # 이미지를 numpy 배열로 변환
                image = np.asanyarray(color_frame.get_data())
                
                # 이미지 그레이스케일로 변환
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                
                # 가장 밝은 부분(흰색에 가까운)을 찾기 위해 이진화
                _, binary = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)
                
                # 밝은 영역 중에서 x값이 min_x_threshold 이상, max_x_threshold 이하,
                # y값이 min_y_threshold 이상, max_y_threshold 이하인 곳 찾기
                bright_points = np.column_stack(np.where(binary == 255))
                bright_points = bright_points[
                    (bright_points[:, 1] >= min_x_threshold) &
                    (bright_points[:, 1] <= max_x_threshold) &
                    (bright_points[:, 0] >= min_y_threshold) &
                    (bright_points[:, 0] <= max_y_threshold)
                ]
                
                if len(bright_points) > 0:
                    max_y = np.max(bright_points[:, 0])
                    y_coordinates.append(max_y)
                    # 원본 이미지에 중심점 그리기
                    cv2.circle(image, (bright_points[bright_points[:, 0] == max_y][0][1], max_y), 5, (0, 0, 255), -1)
                
                # 결과 이미지 보여주기
                cv2.imshow('RealSense Calibration', image)
                
                # 종료 조건 확인
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            # 가장 많이 나온 y좌표 계산
            if y_coordinates:
                most_common_y = Counter(y_coordinates).most_common(1)[0][0]
                pixeld = most_common_y
                print(f"캘리브레이션 완료: 가장 밝은 영역의 최대 y좌표는 {pixeld}입니다.")
            else:
                pixeld = None
                print("밝은 영역을 감지하지 못했습니다.")
                
        except Exception as e:
            print(f"캘리브레이션 중 오류 발생: {e}")
            pixeld = None
        finally:
            # 파이프라인 정리
            pipeline.stop()
            cv2.destroyAllWindows()
            
        return pixeld


def start_socket(server_ip, server_port, max_retries=5, retry_delay=5):
    global UR, gripper, client_socket, server_socket, posx_Objexp_list
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
                    
                    if command == 'Grip1':
                        PickAndGotoMid("outer", 1)
                    elif command == 'Grip2':
                        PickAndGotoMid("outer", 2)
                    elif command == 'Grip3':
                        PickAndGotoMid("outer", 3)
                    elif command == 'Load':  # 기존의 Place를 Load로 변경
                        Load()
                    elif command == 'Unload':  # 새로운 Unload 명령 추가
                        Unload()
                    elif command == 'Move1':  # 새로운 Move 명령 추가
                        MoveToObjectPosition(1)
                    elif command == 'Move2':
                        MoveToObjectPosition(2)
                    elif command == 'Move3':
                        MoveToObjectPosition(3)
                    elif command == 'Cam':
                        UR.movej(posj_Detect_pose)
                        num_objects = 0
                        max_attempts = 10  # 최대 10번 시도
                        attempt = 0
                        
                        while num_objects != 3 and attempt < max_attempts and not isInterrupted:
                            attempt += 1
                            print(f"물체 감지 시도 {attempt}/{max_attempts}")
                            posx_Objexp_list = realsense.show_camera_feed()
                            num_objects = len(posx_Objexp_list)
                            print(f"감지된 물체 수: {num_objects}")
                            
                            if not isInterrupted:
                                try:
                                    client_socket.send(str(num_objects).encode('utf-8'))
                                    # 잠시 대기하여 시스템 안정화
                                    time.sleep(0.5)
                                except Exception as e:
                                    print(f"클라이언트 통신 오류: {e}")
                                    break
                        
                        if num_objects != 3:
                            print(f"경고: {max_attempts}번 시도 후에도 정확히 3개의 물체를 감지하지 못했습니다.")
                    elif command == 'Open':
                        gripper.open()
                    elif command == 'Close':
                        gripper.close()
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
def PickAndGotoMid(option, num):
    global UR, gripper, posx_Objexp_list
    if isInterrupted or UR.prog_stopped:
        return

    if option == "inner":
        posx_Obj = [posx_Obj1, posx_Obj2, posx_Obj3][num - 1]
        posx_Obj_high = [posx_Obj1_high, posx_Obj2_high, posx_Obj3_high][num - 1]
    elif option == "outer":
        # 카메라로 감지된 물체 좌표 사용
        if str(num) not in posx_Objexp_list:
            return
            
        ur_coord = posx_Objexp_list[str(num)]['ur_coord']
        angle_rad = - posx_Objexp_list[str(num)]['angle_rad']
        if angle_rad > 1.57:
            actual_angle = angle_rad - 4.71
        elif angle_rad < -1.57:
            actual_angle = angle_rad + 1.57
        else:
            actual_angle = angle_rad - 1.57
        posx_Obj = [ur_coord[0], ur_coord[1], 412.5, -3.14, 0, actual_angle]
        posx_Obj_high = posx_Obj.copy()
        posx_Obj_high[2] = TABLE_EXT_HEIGHT
    else:
        return

    # 1단계: 안전 경유점을 통해 물체로 이동
    waypoints_to_obj = [posx_Table_high, posx_Obj_high, posx_Obj]
    UR.movel(waypoints_to_obj, "abs")
    # UR.movel([posx_Table_high], "abs")
    # UR.movel([posx_Obj_high], "abs")
    # UR.movel([posx_Obj], "abs")

    
    # 2단계: 그리퍼로 물체 잡기
    gripper.close()
    # UR.scene.attach_object('box', link='tool0')
    
    # 3단계: 물체와 함께 high 위치로 올라가기    
    # 4단계: 테이블 high 경유점을 거쳐 Mid 위치로 이동
    waypoints_to_mid = [posx_Obj_high, posx_Table_high, posx_Mid]
    UR.movel(waypoints_to_mid, "abs")
    # UR.movel([posx_Obj_high], "abs")
    # UR.movel([posx_Table_high], "abs")
    # UR.movel([posx_Mid], "abs")

def Load():
    global UR, gripper
    if isInterrupted or UR.prog_stopped:
        return
    pixeld = realsense.calibrate()
    cali_result_offsetx = -15 * ((272 - pixeld) / (272 - 226))
    print(f"계산된 X축 오프셋: {cali_result_offsetx:.2f}mm")
    
    # # 오프셋을 적용한 새 좌표 생성
    modified_posx_Machine_high = posx_Machine_high.copy()
    modified_posx_Machine = posx_Machine.copy()
    modified_posx_Machine_high[0] += cali_result_offsetx
    modified_posx_Machine[0] += cali_result_offsetx

    # 1단계: Mid 위치에서 Machine_high 위치로 이동
    waypoints_to_machine = [modified_posx_Machine_high, modified_posx_Machine]
    UR.movel(waypoints_to_machine, "abs")
    # UR.movel([modified_posx_Machine_high], "abs")
    # UR.movel([modified_posx_Machine], "abs")
    
    # 3단계: 그리퍼 열어서 물체 배치
    gripper.open()
    # UR.scene.remove_attached_object('tool0', name='box')
    
    # 4단계: Machine에서 Machine_high로 상승
    waypoints_to_mid2 = [modified_posx_Machine_high,posx_Mid]
    UR.movel(waypoints_to_mid2, "abs")
    # UR.movel([modified_posx_Machine_high], "abs")
    # UR.movel([posx_Mid], "abs")


def Unload():
    global UR, gripper
    if isInterrupted or UR.prog_stopped:
        return
    # 1단계: Mid 위치에서 Machine_high 위치로 이동
    # waypoints_to_unload = [posx_Mid, posx_Machine_high, posx_Machine]
    # UR.movel(waypoints_to_unload, "abs")
    UR.movel([posx_Mid], "abs")
    UR.movel([posx_Machine_high], "abs")
    UR.movel([posx_Machine], "abs")
    
    # 3단계: 그리퍼 닫아서 물체 집기
    gripper.close()
    # UR.scene.attach_object('box', link='tool0')
    
    # 4단계: Machine에서 Machine_high로 상승
    waypoints_to_unload2 = [posx_Machine_high, posx_Mid]
    # UR.movel(waypoints_to_unload2, "abs")
    UR.movel([posx_Machine_high], "abs")
    UR.movel([posx_Mid], "abs")

def MoveToObjectPosition(num):
    global UR
    if isInterrupted or UR.prog_stopped:
        return
    
    # 대상 위치 선택
    if num == 1:
        posx_Obj_high = posx_Obj1_high
        posx_Obj = posx_Obj1
    elif num == 2:
        posx_Obj_high = posx_Obj2_high
        posx_Obj = posx_Obj2
    elif num == 3:
        posx_Obj_high = posx_Obj3_high
        posx_Obj = posx_Obj3
    else:
        return

    pixeld = realsense.calibrate()
    cali_result_offsetx = -15 * ((272 - pixeld) / (272 - 226))
    print(f"계산된 X축 오프셋: {cali_result_offsetx:.2f}mm")

    # 1단계: 안전 경유점을 통해 물체 위치로 이동
    
    # 현재 위치가 Mid 주변이라면 테이블 경유점을 거침
    
    # waypoints_to_obj = [posx_Obj_high, posx_Obj]
    # UR.movel(waypoints_to_obj, "abs")
    modified_posx_Obj_high = posx_Obj_high.copy()
    modified_posx_Obj = posx_Obj.copy()
    modified_posx_Obj_high += cali_result_offsetx
    modified_posx_Obj += cali_result_offsetx
    UR.movel([posx_Obj_high], "abs")
    UR.movel([posx_Obj], "abs")
    
    gripper.open()
    # UR.scene.remove_attached_object('tool0', name='box')

    # waypoints_to_table = [posx_Obj_high, posx_Mid]
    # UR.movel(waypoints_to_table, "abs")
    UR.movel([posx_Obj_high], "abs")
    UR.movel([posx_Mid], "abs")

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
    # UR = Move_Group()
    UR = Move_Group(ignore_stop=True) # 기존엔 UR 로봇이 정지 중일 때 코드가 중단된다면, ignore_stop을 사용 시 로봇은 정지 중에도 코드가 작동함.
    UR.movej(posj_Detect_pose)
    gripper = Gripper()
    try:
        # UR.add_obstacle('Table',[-400, -300, -300, 400, 300, 0])
        # UR.add_obstacle('Workspace',[500, 350, -300, 1100, -250, 200])
        # UR.add_obstacle('Machine',[1000, -600, -300, -700, -2500, 1200])
        gripper.open()
        start_socket(Server_IP,Server_Port)
        pass
    except Exception as e:
        print(f"오류가 발생했습니다. - : {e}")
    finally:
        if not isInterrupted:
            shutdown()

def shutdown():                 # 안전종료 메서드
    global UR, client_socket, server_socket
    # UR.scene.remove_world_object('box')        
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

