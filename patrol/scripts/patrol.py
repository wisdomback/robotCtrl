#!/usr/bin/env python3
import rospy
from pynput import keyboard
import signal
import actionlib
import time
import sys

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints = [[
    [(-5, -6, 0), (0, 0, 0, 1)], 
    [(-1, -6, 0), (0, 0, 0, 1)], 
    [(-1, 6, 0), (0, 0, 0, 1)], 
    [(-1, -6, 0), (0, 0, 0, 1)], 
    [(-5, 0, 0), (0, 0, 0, 1)]],
    [
    [(-5, -5, 0), (0, 0, 0, 1)], 
    [(-9.5, -5, 0), (0, 0, 0, 1)], 
    [(-9.5, -1, 0), (0, 0, 0, 1)], 
    [(-9.5, -5, 0), (0, 0, 0, 1)], 
    [(-5, -3, 0), (0, 0, 0, 1)],
    [(-5, 0, 0), (0, 0, 0, 1)]],
    [
    [(-5, -7, 0), (0, 0, 0, 1)], 
    [(-1, -7, 0), (0, 0, 0, 1)], 
    [(-1, 5.5, 0), (0, 0, 0, 1)], 
    [(4, 5.5, 0), (0, 0, 0, 1)], 
    [(-1, 5.5, 0), (0, 0, 0, 1)],
    [(-1, -7, 0), (0, 0, 0, 1)],
    [(-5, -7, 0), (0, 0, 0, 1)],
    [(-5, 0, 0), (0, 0, 0, 1)]]]

# 좌표와 방향 정보를 MoveBaseAction 타입에 맞게 변환하는 함수
def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose

def on_press(key):
    global route
    global count
    try:
        if key.char == '0':
            client.cancel_all_goals()
            route = 0
            count = 0
            print("첫 번째 경로 선택")
        elif key.char == '1':
            client.cancel_all_goals()
            route = 1
            count = 0
            print("두 번째 경로 선택")
        elif key.char == '2':
            client.cancel_all_goals()
            route = 2
            count = 0
            print("세 번째 경로 선택")
        else : print("잘못된 입력입니다.")
    except AttributeError: print("잘못된 입력입니다.")
        

def handler(signum, frame):
    print("Ctrl+C 신호를 수신했습니다.")
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('patrol')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)# 액션 클라이언트 객체 생성
    client.wait_for_server() # 액션 서버가 활성화될 때까지 대기

    route = 0 # 첫 번째 경로 선택
    count = 0 # 초기 위치 설정

    listener = keyboard.Listener(on_press=on_press, on_release=None)
    listener.start()

    while True:
        print(waypoints[route][count]) # 현재 위치 출력
        goal = goal_pose(waypoints[route][count]) # 현재 위치에 해당하는 목표 생성
        client.send_goal(goal) # 액션 클라이언트가 액션 서버에게 목표 전송
        client.wait_for_result() # 액션 서버가 목표 수행할 때까지 대기
        count += 1 # 위치 증가

        if((count-1) == waypoints[route].index([(-5, 0, 0), (0, 0, 0, 1)])):
            print("경로 다시 돌기")
            count = 0        

        time.sleep(3) # 3초 동안 일시 정지
        signal.signal(signal.SIGINT, handler)
