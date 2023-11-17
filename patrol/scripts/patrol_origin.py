#!/usr/bin/env python3

import rospy
import actionlib
import signal
import sys
import time
import termios
import tty
import select

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints = [[
    [(-5, -3, 0), (0, 0, 0, 1)],
    [(-5, -6, 0), (0, 0, 0, 1)], 
    [(-3, -6, 0), (0, 0, 0, 1)],
    [(-1, -6, 0), (0, 0, 0, 1)], 
    [(-1, -3, 0), (0, 0, 0, 1)], 
    [(-1, 0, 0), (0, 0, 0, 1)],
    [(-1, 3, 0), (0, 0, 0, 1)], 
    [(-1, 6, 0), (0, 0, 0, 1)],
    [(-1, 3, 0), (0, 0, 0, 1)], 
    [(-1, 0, 0), (0, 0, 0, 1)],
    [(-1, -3, 0), (0, 0, 0, 1)],
    [(-1, -6, 0), (0, 0, 0, 1)],
    [(-3, -6, 0), (0, 0, 0, 1)],
    [(-5, -6, 0), (0, 0, 0, 1)],
    [(-5, -3, 0), (0, 0, 0, 1)],
    [(-5, 0, 0), (0, 0, 0, 1)]], # 경로 0
    [
    [(-5, -3, 0), (0, 0, 0, 1)],
    [(-5, -5, 0), (0, 0, 0, 1)], 
    [(-7, -5, 0), (0, 0, 0, 1)], 
    [(-9.5, -5, 0), (0, 0, 0, 1)], 
    [(-9.5, -3, 0), (0, 0, 0, 1)], 
    [(-9.5, -1, 0), (0, 0, 0, 1)], 
    [(-9.5, -3, 0), (0, 0, 0, 1)],
    [(-9.5, -5, 0), (0, 0, 0, 1)], 
    [(-7, -5, 0), (0, 0, 0, 1)], 
    [(-5, -3, 0), (0, 0, 0, 1)],
    [(-5, 0, 0), (0, 0, 0, 1)]], # 경로 1
    [
    [(-5, -3, 0), (0, 0, 0, 1)],
    [(-5, -5, 0), (0, 0, 0, 1)],
    [(-5, -7, 0), (0, 0, 0, 1)], 
    [(-3, -7, 0), (0, 0, 0, 1)], 
    [(-1, -7, 0), (0, 0, 0, 1)],
    [(-1, -5, 0), (0, 0, 0, 1)],
    [(-1, -3, 0), (0, 0, 0, 1)],
    [(-1, -1, 0), (0, 0, 0, 1)],
    [(-1, 1, 0), (0, 0, 0, 1)],
    [(-1, 3, 0), (0, 0, 0, 1)],     
    [(-1, 5.5, 0), (0, 0, 0, 1)],
    [(2, 5.5, 0), (0, 0, 0, 1)],
    [(4, 5.5, 0), (0, 0, 0, 1)],
    [(2, 5.5, 0), (0, 0, 0, 1)],
    [(-1, 5.5, 0), (0, 0, 0, 1)],
    [(-1, 3, 0), (0, 0, 0, 1)],
    [(-1, 1, 0), (0, 0, 0, 1)],
    [(-1, -1, 0), (0, 0, 0, 1)],
    [(-1, -3, 0), (0, 0, 0, 1)],
    [(-1, -5, 0), (0, 0, 0, 1)],
    [(-1, -7, 0), (0, 0, 0, 1)],
    [(-3, -7, 0), (0, 0, 0, 1)], 
    [(-5, -7, 0), (0, 0, 0, 1)],
    [(-5, -5, 0), (0, 0, 0, 1)],
    [(-5, -3, 0), (0, 0, 0, 1)],
    [(-5, 0, 0), (0, 0, 0, 1)]]] # 경로 2

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

def getkey():
    fd = sys.stdin.fileno()
    attr = termios.tcgetattr(fd)

    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        # termios.tcsetattr(fd, termios.TCSAFLUSH, attr)
        termios.tcsetattr(fd, termios.TCSANOW, attr)
        
    return ch

if __name__ == '__main__':
    rospy.init_node('patrol')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    old_settings = termios.tcgetattr(sys.stdin)

    route = 0
    count = 0
    
    try:
        tty.setcbreak(sys.stdin.fileno())

        route = 0
        count = 0
        while True:
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                c = sys.stdin.read(1)
                if c == '\x30': # x30 = 0
                    print("0번 경로 전환...")
                    route = 0
                elif c == '\x31':
                    print("1번 경로 전환...")
                    route = 1
                elif c == '\x32':
                    print("2번 경로 전환...")
                    route = 2
                else:
                    print("잘못된 입력입니다.")
                    continue

            print(waypoints[route][count])
            goal = goal_pose(waypoints[route][count])
            client.send_goal(goal)
            client.wait_for_result()
            count += 1

            if((count-1) == waypoints[route].index([(-5, 0, 0), (0, 0, 0, 1)])):
                print("경로 다시 돌기")
                count = 0

            time.sleep(3)
            signal.signal(signal.SIGINT, handler)
    
    finally: termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)