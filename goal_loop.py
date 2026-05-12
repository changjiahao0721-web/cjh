#!/usr/bin/env python3
#coding:utf-8
from geometry_msgs.msg import PoseStamped, Pose
import rospy
import string
import math
import time
import sys
import csv
import os
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64, Int8
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID
import subprocess

import roslaunch

#repeat the route use for test 
#功能:�???�???导航 
#csv要求 一�???�???点是起点下一�???点，终点�???起点 1230
class MultiGoals:
    def __init__(self, goalListX, goalListY, retry, map_frame):
        self.retry = 1
        if(self.retry == 1):
            #self.sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.statusCB, queue_size=1)
            self.pub = rospy.Publisher(
                'move_base_simple/goal', PoseStamped, queue_size=10)
            #self.pose_ekf = rospy.Subscriber('/odometry/filtered',Odometry,self.getPose_ekf,queue_size=10)
            self.pose_amcl = rospy.Subscriber(
                "/amcl_pose", PoseWithCovarianceStamped, self.getPose_amcl, queue_size=10)
            self.start = rospy.Publisher('start', Int8, queue_size=1)
            # params & variables
            self.pub_final = rospy.Publisher('/arrfinal', Float64, queue_size=1)
            self.goalListX = goalListX
            self.goalListY = goalListY
            self.goalListW = goalListW
            self.goalListZ = goalListZ
            self.kx = 0
            self.ky = 0
            self.gx = 0
            self.gy = 0
            self.x = 0
            self.y = 0
            self.z = 0
            self.w = 0
            self.once_stop=False
            self.flag = 1
            self.MIN_DISTANCE = 0.6  # min distance of the judge between the goal and odometrypose
            self.LONG = len(self.goalListX)
            self.goalId = 0
            self.count = 0
            self.start_time = 0
            self.pubfinal = True
            self.goalMsg = PoseStamped()
            self.goalMsg.header.frame_id = map_frame
            self.second=False
            self.Stop=False
            time.sleep(1)
            self.goalMsg.header.stamp = rospy.Time.now()
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.orientation.z = self.goalListZ[self.goalId]
            self.goalMsg.pose.orientation.w = self.goalListW[self.goalId]
            # self.x = self.goalMsg.pose.position.x
            # self.y = self.goalMsg.pose.position.y
            # self.z = self.goalMsg.pose.position.z
            # self.w = self.goalMsg.pose.position.w
            self.pub.publish(self.goalMsg)
            # print(self.goalMsg)
            self.start_time = rospy.get_time()
            # rospy.loginfo(
            #     "Initial goal published! Goal ID is: %d", self.goalId)
            self.goalId = self.goalId + 1

    def statusCB(self):
        if self.pubfinal == True:

            self.gx = self.goalListX[self.goalId-1] if(self.goalId != 0) else self.goalListX[self.goalId]
            self.gy = self.goalListY[self.goalId-1] if(self.goalId != 0) else self.goalListY[self.goalId]

            # self.dist = self.distance(self.kx, self.ky, self.gx, self.gy)
            # # if abs(self.kx-self.gx)<0.5 and abs(self.ky-self.gy)<0.5 :
            # print(abs(self.kx-self.gx),abs(self.ky-self.gy))
            if abs(self.kx-self.gx)<2 and abs(self.ky-self.gy)<2 and not self.Stop:
                
                # if self.gx==-4 and self.count>5:
                #     pass
                #     # time.sleep(3)
                # if self.gx==-0.5 and self.count>5:
                #     pass
                #     # self.pubfinal=False
                #     # exit(0)

                # finish_time = rospy.get_time()
                # interval = finish_time - self.start_time
                # print(interval)
                if self.goalId == self.LONG:
                    self.goalId = 0
                self.goalMsg.header.stamp = rospy.Time.now()
                self.goalMsg.pose.position.x = self.goalListX[self.goalId]
                self.goalMsg.pose.position.y = self.goalListY[self.goalId]
                self.goalMsg.pose.orientation.z = self.goalListZ[self.goalId]
                self.goalMsg.pose.orientation.w = self.goalListW[self.goalId]
                if self.goalMsg.pose.position.x==0:
                    file=open("/racecar/1.txt","w")
                self.pub.publish(self.goalMsg)
                # rospy.loginfo(
                #     "Initial goal published! Goal ID is: %d", self.goalId)
                # rospy.loginfo("intostatusCB")
                self.count = self.count+1
                if self.count==3:
                    open=Int8()
                    open.data=1
                    self.start.publish(open)   #rviz启动
                # print(self.count)
                # if self.count>5 and not self.once_stop :
                #     self.second=True
                #     self.once_stop=True
                #     print("-----------------------")
                if self.goalId < (len(self.goalListX)):
                    self.goalId = self.goalId + 1

                else:
                    self.goalId = 0
                    # print("final")


    def getPose_amcl(self, data):
        self.kx = data.pose.pose.position.x
        self.ky = data.pose.pose.position.y
        # print(abs(self.kx+10),abs(self.ky))
        # if abs(self.kx+8) <0.5 and self.ky<0.5 and self.second:
            # pass
            # # cancel_msg = GoalID()
            # print("-----------------------")
            # # cancel_msg.id = ''  
            
            # self.second=False
            # # self.once_stop=True
            # # rospy.sleep(3)
            # self.Stop=True
            # # os.system("rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}")
            # process = subprocess("rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}")
            # pid = process.pid
            # # time.sleep(60)
            # os.kill(pid,signal.CTRL_C_EVENT)
            # # start_time = rospy.get_time()
            # # while rospy.get_time()-start_time <3:
            # #     # print(rospy.get_time()-start_time)
            # #     pass
            # #     # self.cancel_pub.publish(cancel_msg)
            # self.pub.publish(self.goalMsg)
            # self.Stop=False
        self.statusCB()

    def distance(self, kx, ky, gx, gy):
        try:
            return math.sqrt((kx-gx)**2+(ky-gy)**2)
        except:
            return None



if __name__ == "__main__":
    try:
        # ROS Init
        rospy.init_node('multi_goals', anonymous=True)
        retry = 1
        goalList = []
        goalListX=[]
        goalListY=[]
        goalListZ=[]
        goalListW=[]
        map_frame = rospy.get_param('~map_frame', 'map' )

        # if round == 1:
        with open('/racecar/src/racecar/scripts/test_01.csv', 'r') as f:
            reader = csv.reader(f)
            for cols in reader:
                goalList.append([float(value) for value in cols])

        goalList = np.array(goalList)
        print(goalList)
        print("read first round suc!!")
        goalListX = goalList[:,0]
        goalListY = goalList[:,1]
        goalListZ = goalList[:,2]
        goalListW = goalList[:,3]
        #         round = round + 1
        
        # elif round ==2 :
        #     with open('/home/racecar/racecar/src/racecar/src/test_02.csv', 'r') as f:
        #         reader = csv.reader(f)
        #         for cols in reader:
        #             goalList.append([float(value) for value in cols])

        #         goalList = np.array(goalList)
        #         print(goalList)
        #         print("read second round suc!!")
        #         goalListX = goalList[:,0]
        #         goalListY = goalList[:,1]
        #         goalListZ = goalList[:,2]
        #         goalListW = goalList[:,3]
        #         round = 1
        # while True:
            
        #     if round == 1:  
        #         filename = 'test_01.csv'  
        #     elif round == 2:  
        #         filename = 'test_02.csv'  
        #     else:  
        #         print("two rounds suc!!!  Invalid round value. Only 0 or 1 is allowed.")  
        #         exit()  
        #     with open('/home/racecar/racecar/src/racecar/src/' + filename, 'r') as f:  
        #         reader = csv.reader(f)  
        #         print("read   "+ str(round) + "   round suc!!")
        #         for cols in reader: 
        #             goalList.append([float(value) for value in cols])  
        #     goalList = np.array(goalList)  
        #     print(goalList)   
        #     goalListX = goalList[:,0]  
        #     goalListY = goalList[:,1]  
        #     goalListZ = goalList[:,2]  
        #     goalListW = goalList[:,3]  
        #     round = round + 1

        if len(goalListX) == len(goalListY) & len(goalListY) >= 1:
            # Constract MultiGoals Obj
            rospy.loginfo("Multi Goals Executing...")
            mg = MultiGoals(goalListX, goalListY, retry, map_frame)
            rospy.spin()
        else:
            rospy.loginfo("Lengths of goal lists are not the same")
    except KeyboardInterrupt:
        print("shutting down")


