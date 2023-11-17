#!/usr/bin/env python3

import rospy
import actionlib
import signal
import sys
import time
from getkey import getkey
import threading
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

def handler(signum, frame):
    print("Ctrl+C 신호를 수신했습니다.")
    sys.exit(0)

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

if __name__ == '__main__':
    rospy.init_node('patrol')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    count = 0
    route = 0

    while True:
        print(waypoints[route][count])
        goal = goal_pose(waypoints[route][count])
        client.send_goal(goal)

        route = int(input())
        if route == 0: 
            client.cancel_all_goals()
            print("0번 경로 변경...")
        elif route == 1: 
            client.cancel_all_goals()
            print("1번 경로 변경...")
        elif route == 2: 
            client.cancel_all_goals()
            print("2번 경로 변경...")
        else:
            client.cancel_all_goals()
            print("잘못된 입력입니다. 기본(0)으로 전환합니다.")
            route = 0
        client.wait_for_result()
        count += 1

        if((count-1) == waypoints[route].index([(-5, 0, 0), (0, 0, 0, 1)])):
            print("경로 다시 돌기")
            count = 0

        time.sleep(3)
        signal.signal(signal.SIGINT, handler)

# if __name__ == '__main__':
#     rospy.init_node('patrol')
#     client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#     client.wait_for_server()

#     count = 0
#     route = 0

#     while True:
#         check = getkey()
#         keyThread = threading.Thread(target=ChangeRoute, args=(check,))
#         keyThread.start()

#         print(waypoints[route][count])
#         goal = goal_pose(waypoints[route][count])
#         client.send_goal(goal)
#         client.wait_for_result()
#         count += 1

#         if((count-1) == waypoints[route].index([(-5, 0, 0), (0, 0, 0, 1)])):
#             print("경로 다시 돌기")
#             count = 0

#         time.sleep(3)
#         signal.signal(signal.SIGINT, handler)
#         keyThread.join()