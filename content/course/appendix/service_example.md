---
title: サービス通信の例(仮)
date: '2023-08-27'
type: book
weight: 2
---
serviceを用いたnode間通信の例を挙げます。

task_manager.pyで起動したノードから、
serviceを用いてmove_robot.pyで起動したノードの機能を使用する例です。


## pythonファイルの用意
navigation_tutorial/scripts/に以下の2つのファイルを作成してください
(コピペで構いません)。

task_manager.py
```
#!/usr/bin/env python3
import rospy
from navigation_tutorial.srv import MoveTrigger, MoveTriggerRequest

class TaskManager:
    def __init__(self):
        rospy.init_node('task_manager')

        # service client
        self.move_robot = rospy.ServiceProxy('/move_robot', MoveTrigger)

        self.move_robot.wait_for_service()
        rospy.loginfo("Service /move_robot is ready!")

    def move(self, straight, turn):
        command = MoveTriggerRequest()
        command.straight = straight
        command.turn = turn
        rospy.loginfo(f"request: straight={straight}, turn={turn}")
        response = self.move_robot(command) # service call
        return response.success

    def main(self):
        """
        タスクの流れを手続的に記述する
        """
        result = self.move(1.0, 0.0)
        if result:
          rospy.loginfo("move success!")
        else:
          rospy.loginfo("move failed!")
        
        result = self.move(0.0, 90.0)
        if result:
          rospy.loginfo("turn success!")
        else:
          rospy.loginfo("turn failed!")

        rospy.loginfo("task completed!")

if __name__ == '__main__':
    task_manager = TaskManager()
    task_manager.main()

```
<br>
<br>

move_robot.py
```
#!/usr/bin/env python3
import numpy as np

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from navigation_tutorial.srv import MoveTrigger, MoveTriggerResponse

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)

        # Service Server
        rospy.Service('/move_robot', MoveTrigger, self.callback_move_robot)

        self.x = None
        self.y = None
        self.yaw = None
        while self.x is None:
            rospy.sleep(0.1)

    def callback_move_robot(self, req):
        res = MoveTriggerResponse()
        try:
            self.go_straight(req.straight)
            if req.turn >= 0:
                self.turn_left(req.turn)
            else:
                self.turn_right(-req.turn)
            res.success = True
            return res
        except rospy.ROSInterruptException:
            res.success = False
            return res

    def callback_odom(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.yaw = self.get_yaw_from_quaternion(data.pose.pose.orientation)

    def go_straight(self, dis, velocity=0.3):
        vel = Twist()
        x0 = self.x
        y0 = self.y
        while(np.sqrt((self.x-x0)**2+(self.y-y0)**2)<dis):
            vel.linear.x = velocity
            vel.angular.z = 0.0
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        self.stop()

    def turn_right(self, yaw, yawrate=-0.5):
        vel = Twist()
        yaw0 = self.yaw
        while(abs(self.yaw-yaw0)<np.deg2rad(yaw)):
            vel.linear.x = 0.0
            vel.angular.z = yawrate
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        self.stop()

    def turn_left(self, yaw, yawrate=0.5):
        vel = Twist()
        yaw0 = self.yaw
        while(abs(self.yaw-yaw0)<np.deg2rad(yaw)):
            vel.linear.x = 0.0
            vel.angular.z = yawrate
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        self.stop()

    def stop(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)

    def get_yaw_from_quaternion(self, quaternion):
        e = tf.transformations.euler_from_quaternion(
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return e[2]

if __name__=='__main__':
    simple_controller = SimpleController()
    rospy.spin()

```
move_robot.pyはsimple_control2.pyを少し修正した程度です。
simple_control2.pyについては、[chapter3](../../chap3/sensing2)で詳しく解説しています。

## srvファイルの用意
また、[このページ](https://note.com/npaka/n/n3e90d24bd38b)を参考にして、
navigation_tutorialパッケージ内に
navigation_tutorilal/srv/MoveTrigger.srvファイルを作成してください。

とくに、navigation_tutorial/CMakeLists.txtを編集する箇所を抜かさないように気をつけてください
（[対応する説明](https://note.com/npaka/n/n3e90d24bd38b#:~:text=(2)%20%E3%80%8CCMAkeLists.txt%E3%80%8D%E3%82%92%E4%BB%A5%E4%B8%8B%E3%81%AE%E3%82%88%E3%81%86%E3%81%AB%E7%B7%A8%E9%9B%86%E3%80%82)）。

MoveTrigger.srv
```
float32 straight
float32 turn
---
bool success
```

## 実行方法
ターミナルで以下のコマンドを実行した後、
```
$ rosrun navigation_tutorial move_robot.py
```

別のターミナルで
```
$ rosrun navigation_tutorial task_manager.py
```
を実行すると、ロボットが直進して左に回転します。

## 補足
ファイルがうまく実行できない場合、ファイルに実行権限があたえられていない可能性があります。
[このページ](https://qiita.com/shisama/items/5f4c4fa768642aad9e06)を参考にして解決してみてください。

それでも解決しない場合はslackで気軽に質問してください。
