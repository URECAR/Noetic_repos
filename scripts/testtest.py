#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import tf.transformations

def cartesian_path_example():
    # 초기화
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cartesian_path_example', anonymous=True)

    # 로봇 제어에 필요한 객체 생성
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"  # 로봇 그룹 이름 (예: UR의 경우 "manipulator")
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    # 현재 로봇 상태 출력
    print("============ 로봇 참조 프레임: %s" % move_group.get_planning_frame())
    print("============ 엔드 이펙터 링크: %s" % move_group.get_end_effector_link())
    print("============ 사용 가능한 계획 그룹: %s" % ", ".join(robot.get_group_names()))
    print("============ 현재 로봇 상태:")
    print(robot.get_current_state())
    print("============")

    # 카테시안 경로 계획 및 실행
    cartesian_move(move_group)

    # 종료
    moveit_commander.roscpp_shutdown()

def cartesian_move(move_group):
    # 카테시안 경로를 위한 웨이포인트 리스트 생성
    waypoints = []
    
    # 현재 엔드 이펙터 위치를 시작점으로 사용
    wpose = move_group.get_current_pose().pose
    
    # 1. X 방향으로 10cm 이동
    wpose.position.x += 0.1
    waypoints.append(copy.deepcopy(wpose))
    
    # 2. Z 방향으로 10cm 상승
    wpose.position.z += 0.1
    waypoints.append(copy.deepcopy(wpose))
    
    # 3. Y 방향으로 10cm 이동
    wpose.position.y += 0.1
    waypoints.append(copy.deepcopy(wpose))
    
    # 카테시안 경로 계산
    # eef_step: 엔드 이펙터의 최대 이동 거리 (단위: 미터)
    # jump_threshold: 연속적인 지점들 간의 각도 변화 허용 범위 (0은 제한 없음)
    (plan, fraction) = move_group.compute_cartesian_path(
                                waypoints,   # 웨이포인트
                                0.01,        # eef_step
                                0.0)         # jump_threshold
    
    # 계획의 성공 여부 출력
    print("계획된 경로 비율: %s" % fraction)
    
    # 계획이 성공적이면 로봇 이동 실행
    if fraction > 0.9:  # 90% 이상 경로가 계획되면 실행
        print("카테시안 경로 실행 중...")
        move_group.execute(plan, wait=True)
        print("이동 완료")
    else:
        print("경로 계획 실패: 목표 경로의 %s%% 만 계획됨" % (fraction * 100))

if __name__ == '__main__':
    try:
        cartesian_path_example()
    except rospy.ROSInterruptException:
        print("프로그램 종료")
    except Exception as e:
        print("오류 발생: %s" % e)