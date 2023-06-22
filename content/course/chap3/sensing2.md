---
title: ロボットシステムにおけるセンシング・アクチュエーション・通信②
date: '2022-04-05'
type: book
weight: 20
---

複数のセンサを組み合わせてよりかしこくロボットを動かしてみよう
<!--more-->

## Learn

前回使用した[simple_control.py](https://github.com/matsuolab/roomba_hack/blob/master/catkin_ws/src/navigation_tutorial/scripts/simple_control.py)
では，速度，角速度，時間を指定し，
 "速度 * 時間" あるいは "角速度 * 時間" という演算を行うことで，
ロボットを意図した場所へ移動させる命令を与えていました．  

しかし，この制御の仕方には，いくつかの問題があります．

- ホイールと地面との間に滑りがあった場合，ロボットは指定した距離より小さい距離しか移動しない可能性がある．
- ロボット本体の問題で，指定した速度よりも実際の速度が大きいまたは小さい場合，
  ロボットは指定した位置には移動しない可能性がある．  

これらは，動作の結果を考慮せず，はじめに指定した速度と時間にのみ従って動く，という制御の仕方のために起こります．  

このように，あらかじめ指定された制御信号にのみ基づいて制御を行い，
その結果（フィードバック情報）を考慮しない制御の仕方を
{{< hl >}}フィードフォワード制御{{< /hl >}}（開ループ制御）と呼びます．
フィードフォワード制御は，制御対象が予測可能で外乱が少ない場合や，
システムが簡潔である場合に使用されることがあります．

一方で，センサーからのフィードバック情報を利用して制御信号を修正する制御の仕方を
{{< hl >}}フィードバック制御{{< /hl >}}（閉ループ制御）と呼びます．
フィードバック制御は，制御対象の予測が難しく，外乱が大きい場合に有効です．

今回は，ロボットのセンサ情報を用いるフィードバック制御
によってロボットをより柔軟に動かしてみましょう．
オドメトリとLiDARという2種類のセンサの情報を用います．

<!--
前回の演習のようにロボットに速度と時間を一回与えて，その通りに動かすようなフィードフォワード制御ではなく，今回は，ロボットが逐次的にセンサの情報を反映して振る舞いを変える{{< hl >}}フィードバック制御{{< /hl >}}を行なってみましょう．-->


### オドメトリのセンサ情報を用いた制御

まずは，ロボットのタイヤの回転量から計算される移動距離である{{< hl >}}（ホイール）オドメトリ（odometry）{{< /hl >}}を使った制御をしてみましょう．


#### オドメトリのメッセージ（`/odom`）の中身を見てみよう

roombaのオドメトリの情報は，`/odom`トピックにpublishされています．

```
$ rostopic echo /odom
```
を実行するとメッセージとしてどのような情報が流れているかがわかります．
{{< spoiler text="`$ rostopic echo -n 1 /odom`">}}
```bash
root@dynamics:~/roomba_hack# rostopic echo -n 1 /odom
header:
  seq: 2115
  stamp:
    secs: 1649692132
    nsecs: 791056254
  frame_id: "odom"
child_frame_id: "base_footprint"
pose:
  pose:
    position:
      x: -0.014664691872894764
      y: -0.0010878229513764381
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0056752621080531414
      w: 0.9999838955703261
  covariance: [0.08313143998384476, 0.00019857974257320166, 0.0, 0.0, 0.0, 0.004368376452475786, 0.00019857988809235394, 0.015032557770609856, 0.0, 0.0, 0.0, -0.26573312282562256, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0043683769181370735, -0.26573312282562256, 0.0, 0.0, 0.0, 6.021446704864502]
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```
{{< /spoiler >}}

また，
```
$ rostopic type /odom
```
を実行すると，メッセージとして，`nav_msgs/Odometry`型が使われていることがわかります．
{{< spoiler text="`$ rostopic type /odom`">}}
```bash
root@dynamics:~/roomba_hack# rostopic type /odom
nav_msgs/Odometry
```
{{< /spoiler >}}

[nav_msgs/Odometry型のドキュメント](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)を確認すると，このメッセージは`pose`と`twist`で構成されていることがわかります．

- `pose`は（`child_frame`から見た）ロボットの推定姿勢（位置と回転角）を表していて，`covariance`にはその不確かさを表す共分散が記録されています．

- 一方，`twist`は（`child_frame`から見た）ロボットの速度を表していて，`pose`と同様に`covariance`にはその不確かさを表す共分散が記録されています．

なお，メッセージ型の定義は，コマンドで
```
$ rosmsg info nav_msgs/Odometry
```
を実行しても確認できます．
{{< spoiler text="`$ rosmsg info nav_msgs/Odometry`">}}
```bash
root@dynamics:~/roomba_hack# rosmsg info nav_msgs/Odometry
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
```
{{< /spoiler >}}


#### クォータニオン(quaternion)

さて，`/odom`のトピックでは，ロボットの回転角は{{< hl >}}クォータニオン（quaternion）{{< /hl >}}で記述されています．

クォータニオンは，日本語では四元数と呼ばれ，3次元空間上での回転角を表現する方法の一つで，4つの要素を持つベクトルで表現されます．

クォータニオンによる3次元回転の表現は，角度を連続的にかつ簡潔に表現できるためROSではよく用いられます（その他には，オイラー角による表現や回転行列による表現があります）．

それぞれの回転角に関する表現のメリット・デメリットを調べてみましょう（「ジンバルロック」などのキーワードで調べるとよりよく理解できると思います）．

クォータニオンからオイラー角へは，`tf`パッケージの`tf.transformations.euler_from_quaternion`を使うことで変換できます（[ドキュメント](http://docs.ros.org/en/jade/api/tf/html/python/transformations.html#tf.transformations.euler_from_quaternion)）．


#### 実装の例を見てみる

それでは，オドメトリ`/odom`の情報を使った制御の実装の例として`navigation_tutorial`パッケージの中の`simple_control2.py`を見てみましょう（[github](https://github.com/matsuolab/roomba_hack/blob/master/catkin_ws/src/navigation_tutorial/scripts/simple_control2.py)）．


### ソースコードを読んでみよう
画面にウィンドウを2つ並べるなど，[githubのソースコード](https://github.com/matsuolab/roomba_hack/blob/master/catkin_ws/src/navigation_tutorial/scripts/simple_control2.py)をみながら以下の解説を読むことをお勧めします．

{{< spoiler text="simple_control2.py">}}
```python
#!/usr/bin/env python3
import numpy as np

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class SimpleController:
    def __init__(self):
        rospy.init_node('simple_controller', anonymous=True)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)

        self.x = None
        self.y = None
        self.yaw = None
        while self.x is None:
            rospy.sleep(0.1)

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
    try:
        simple_controller.go_straight(1.0)
        simple_controller.turn_left(90)
        simple_controller.turn_right(90)
    except rospy.ROSInitException:
        pass
```
{{< /spoiler >}}

#### かんたんな説明
上記ソースコードの大枠のみを抜き出すと，以下のようになっています．
{{< spoiler text="simple_control2.pyの大枠">}}
```python
#!/usr/bin/env python3  # 1
import ~  # 2

class SimpleController:  # 3
    def __init__(self):  # 4
        pass
    def callback_odom(self, data):  # 5
        pass
    def go_straight(self, dis, velocity=0.3):
        pass
    def turn_right(self, yaw, yawrate=-0.5):
        pass
    def turn_left(self, yaw, yawrate=0.5):
        pass
    def stop(self):
        pass
    def get_yaw_from_quaternion(self, quaternion):
        pass

if __name__=='__main__':  # 6
    simple_controller = SimpleController()  # 7
    pass
``` 
{{< /spoiler >}}

それぞれについて簡潔に解説します．  

{{< spoiler text="#1 shebang">}}
shebang(シバン)と呼ばれるもので，このファイルを実行する際に，どのプログラムを使って実行するかを指定する．
`#!/usr/bin/env python3` と書いてあるので，このファイルはpython3で実行するのだとコンピュータに教えている．
{{< /spoiler >}}

{{< spoiler text="#2 import">}}
```python
import numpy as np

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
```
- pythonの標準の関数(printなど)だけでは機能が足りないので，別のモジュールをインポートしている．
- このソースコードでは，numpy, rospy, tf というモジュールをインポートしている．
    - rospyというのが，pythonでrosを使うためのモジュール．
- また，geometry_msgs.msgというモジュールからTwistというデータ型, nav_msgs.msgというモジュールからOdometoryというデータ型をそれぞれインポートしている．
{{< /spoiler >}}

{{< spoiler text="#3 class">}}
- SimpleControllerという名前のクラスを定義している．
- クラスはオブジェクトの設計図のようなもの．
    - オブジェクトとは，データとそのデータの振る舞いをまとめたもの．

{{< spoiler text="classの例1" >}}
以下のようなクラスを定義したとする．
```python
class Car:
def __init__(self, color, speed):
    self.color = color
    self.speed = speed
    self.fuel = 100

def drive(self):
    self.fuel -= 20
    print('drove!')
    print(f'残りの燃料は{self.fuel}リットルです')

def charge(self):
    self.fuel = 100
    print('charged!')
    print(f'残りの燃料は{self.fuel}リットルです')

def info(self):
    print(f'色は{self.color}です')
    print(f'速度は{self.speed}km/hです')
    print(f'残りの燃料は{self.fuel}リットルです')
```
以下のように使える．
```python
mycar = Car('red', 200)

mycar.drive()
#drove!
#残りの燃料は80リットルです

mycar.drive()
#drove!
#残りの燃料は60リットルです

mycar.charge()
#charged!
#残りの燃料は100リットルです

mycar.info()
#色はredです
#速度は200km/hです
#残りの燃料は100リットルです
```
{{< /spoiler >}}

{{< spoiler text="classの例2" >}}
pythonのstring型や，int型，list型も，実はオブジェクトである．
```python
people = ['Alice', 'Bob', 'Charlie']
people.append('Dave')

print(people)
#['Alice', 'Bob', 'Charlie', 'Dave']
```
上の例では，list型のオブジェクトpeopleに対して，appendというメソッド(そのオブジェクトが持つ関数)を呼び出し，新しい要素を追加している．
{{< /spoiler >}}

{{< /spoiler >}}

{{< spoiler text="#4 コンストラクタ">}}
- コンストラクタ__init__()とは，オブジェクト生成時に呼び出される関数のこと．
- 初期化のための関数というイメージ．
{{< /spoiler >}}

{{< spoiler text="#5 メソッドの定義">}}
- メソッドとは，オブジェクトが持つ関数のこと．
- classの定義の中では，`self.method_name(引数1, 引数2)`という形で呼び出すことができる．
- オブジェクトの外から使用するときには，上のCarの例のように，`object_name.method_name(引数1, 引数2)`という形で呼び出すことができる．
- 定義の第一引数には，必ず`self`を指定する．これは，そのオブジェクト自身を指す．
    - 呼び出すときには`self`は省略する．
{{< /spoiler >}}

{{< spoiler text="#6 ファイル実行時の処理">}}
- このif文の中の処理は，ファイルを直接実行したときにのみ実行される．
- `__name__`は特殊な変数で，ファイルを直接実行したときには`'__main__'`という値を持つ．
    - importされたときには`__name__`にはファイル名が入るため，このif文の中の処理は実行されない．
        ```python
        #!/usr/bin/env python3
        print(__name__)
        ```
        とだけ記述したファイルを実行してみると，ふるまいが理解しやすいかもしれない．
{{< /spoiler >}}

#### より詳細な理解

simple_control2.pyについてより詳細に解説します．  

{{< spoiler text="コンストラクタ">}}
```python
    def __init__(self):
        rospy.init_node('simple_controller', anonymous=True)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)

        self.x = None
        self.y = None
        self.yaw = None
        while self.x is None:
            rospy.sleep(0.1)
```
上にも述べた通り，`__init__`はコンストラクタと呼ばれ，オブジェクト生成時に自動で呼び出される関数です．
各行について順番に見ていきます．
```python
        rospy.init_node('simple_controller', anonymous=True)
```
- `simple_controller`という名前のノードを作成しています．

```python
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
```
- `rospy.Publisher()`によって，上で作成したノードがpublisherとして機能することを宣言しています．
- このノードは，`/cmd_vel`というトピックに対して，`Twist`というデータ型のメッセージを送信しています．
    - `Twist`の情報は，[geometry_msgsのドキュメント](http://wiki.ros.org/geometry_msgs)より確認できます．
    - `Twist`は速度３成分と角速度３成分を格納するデータ型です．
- 第２引数のqueue_sizeは，メッセージを送信する命令が，許容できる周期より短い場合に，メッセージをキューにためておく数を指定します．
    - ここでは，10個までキューにためておくことを指定しています．
    - たまった数が10個より少ない場合には，古い順にメッセージをパブリッシュしていきます．
    - たまった数が10個に達した場合には，最も古いメッセージを破棄します．

```python
        odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)
```
- `rospy.Subscriber()`によって，作成したノードがsubscriberとして機能することを宣言しています．
- このノードは，`/odom`というトピックから，`Odometry`というデータ型のメッセージを受信しています．
    - `Odometry`の情報は，[nav_msgsのドキュメント](http://wiki.ros.org/nav_msgs)より確認できます．
    - `Odometry`は，位置と姿勢，及び速度と角速度を格納するデータ型です．
    - ここでは，`Odometry`には，ルンバの現在の位置や運動の様子が格納されており，それを受信しています．
- rospy.Subscriber()の第三引数として，`self.callback_odom`というコールバック関数を指定しています．
    - subscriberにはコールバック関数を指定する必要があります．
    - コールバック関数とは，subscriberがメッセージを受信したときに実行される関数のことです．
        - コールバック関数は，メッセージを引数として実行します．
    - コールバック関数の中身は，後述します．

```python
        self.x = None
        self.y = None
        self.yaw = None
```
- アトリビュートを定義しています．
- クラスの定義の中で`self.<name>`の形式で表される変数は，そのクラスのオブジェクトが持つ"attribute(アトリビュート)"と呼ばれます．
    - アトリビュートには，そのclassの定義の中であればどこからでもアクセスできます．
- ここでは，`self.x`, `self.y`, `self.yaw`というアトリビュートを定義しています．
    - これらのアトリビュートは，後述するコールバック関数の中で値が更新されます．
    - このアトリビュートには，ロボットの現在の位置や姿勢が格納されます．

```python
        while self.x is None:
            rospy.sleep(0.1)
```
- `self.x`がNoneである間，0.1秒間隔で待機し続けます．

{{< /spoiler >}}

{{< spoiler text="コールバック関数" >}}
```python
    def callback_odom(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.yaw = self.get_yaw_from_quaternion(data.pose.pose.orientation)
```
- コンストラクタの項目で説明した通り，subscriberを定義する際にはコールバック関数を指定する必要があります．
- コールバック関数はsubscriberがメッセージを受信した際，そのメッセージを引数として実行する関数でした．
- 上のコールバック関数では，引数`data`には，`Odometry`型のメッセージが格納されます．
    - `data.pose.pose.position`という書き方によって，Odometry型の中の，位置を表すpositionという要素にアクセスしています．
    - 同様に，`data.pose.pose.orientation`という書き方によって，Odometry型の中の，姿勢を表すorientationという要素にアクセスしています．
- コンストラクタで定義していたアトリビュート`self.x`, `self.y`, `self.yaw`に，メッセージから得られた位置と姿勢を格納しています． 
    - `self.yaw`に値を格納する時に使用している関数`get_yaw_from_quaternion()`については，後述します．
- このコールバック関数が一度でも呼ばれると，`self.x`に入っているNoneの値が上書きされ，コンストラクタの中のwhile文が終了します．
{{< /spoiler >}}

{{< spoiler text="get_yaw_from_quaternion関数" >}}
go_straight関数やturn_right関数とは順番が前後しますが，先にget_yaw_from_quaternion関数について説明します．
```python
    def get_yaw_from_quaternion(self, quaternion):
        e = tf.transformations.euler_from_quaternion(
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        return e[2]
```
- tfモジュールのeuler_from_quaternion関数を利用しています．
    - euler_from_quaternion()は，クォータニオン(4要素)を引数として，オイラー角(3要素)を返す関数です．
    - クォータニオンについてはLearn/オドメトリのセンサ情報を用いた制御/クォータニオンの項目で説明しました．
- `Odometry`型のメッセージのうち，姿勢を表すorientationという要素は，クォータニオンで表されているため，オイラー角で制御したい場合には，この関数を用いてオイラー角に変換する必要があります．
- ルンバはxy平面上を動くため，z軸周りのオイラー角さえわかれば十分です．そのため，このget_yaw_from_quaternion関数ではオイラー角のz軸成分(第2成分)のみを返しています．
{{< /spoiler >}}

{{< spoiler text="go_straight関数" >}}
```python
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
```
ロボットを直進させる関数です．順に説明します

```
        vel = Twist()
```
- `Twist`型のオブジェクトを生成しています．

```python
        x0 = self.x
        y0 = self.y
```
- self.x, self.yには，コールバック関数で更新された(直前の)ルンバの位置が格納されています．それをx0, y0に代入しています．

```python
        while(np.sqrt((self.x-x0)**2+(self.y-y0)**2)<dis):
            vel.linear.x = velocity
            vel.angular.z = 0.0
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
```
- 点(self.x, self.y)と点(x0, y0)の距離が指定したdisより小さい間，while以下の処理を繰り返します．
    - velの並進成分のx成分に，引数で指定したvelocityの値を格納します．
    - velの回転成分のz成分(z軸周りの角速度)に，0を格納します．
    - コンストラクタの中で定義したpublisherである`self.cmd_vel_pub`の.publish()関数を用いて，publishを行います．
        - 引数にvelを指定しているので，確かにTwist型のメッセージをパブリッシュしています．
    - `rospy.sleep(0.1)`で0.1秒待ち，次のループに入ります．

```python
        self.stop()
```
- while文が終了したら，ロボットを停止させます．
    - stop関数については，説明を省略します．
- turn_right, turn_left関数についても，go_straight関数の説明と同様なので省略します．
{{< /spoiler >}}

単純に "(角)速度 * 時間" によって移動の姿勢を指定しているのではなく，オドメトリのセンサ情報を使いながら，
目標の姿勢に到達するように制御していることを再度強調しておきます．

pythonのコードの読み方についての基本的な説明は上の説明で尽きているので，
余力があれば（なくても）各自[roomba_hackリポジトリ](https://github.com/matsuolab/roomba_hack)
上の気になったコードを読んでみましょう．

## 演習

{{< spoiler text="【jetson・開発マシン】それぞれdockerコンテナを起動" >}}．
jetsonでdockerコンテナを起動

```shell
(開発PC):~$ ssh roomba_dev1
(jetson):~$ cd ~/group_a/roomba_hack
(jetson):~/group_a/roomba_hack ./RUN-DOCKER-CONTAINER.sh
(jetson)(docker):~/roomba_hack#  
```

開発PCでdockerコンテナを起動

```shell
(開発PC):~$ cd ~/group_a/roomba_hack
(開発PC):~/group_a/roomba_hack ./RUN-DOCKER-CONTAINER.sh 192.168.10.7x
(開発PC)(docker):~/roomba_hack#
```
{{< /spoiler >}}

{{< spoiler text="【jetson】ROSマスタ，各種ノードを起動" >}}

```shell
(jetson)(docker):~/roomba_hack# roslaunch roomba_bringup bringup.launch
```
{{< /spoiler >}}

### ROSメッセージの可視化
{{< spoiler text="【開発PC】topicの確認" >}}

`/odom`の型を確認

```shell
(開発PC)(docker):~/roomba_hack# rostopic type /odom
```

`/odom`の中身を確認
```shell
(開発PC)(docker):~/roomba_hack# rostopic echo /odom
```
{{< /spoiler >}}

{{< spoiler text="オドメトリを使ったフィードバック制御" >}}


`simple_control2.py`を実行してみよう．

開発PCでteleopのコードを実行しましょう
```shell
(開発PC)(docker):~/roomba_hack# roslaunch roomba_teleop teleop.launch
```

このプログラムを動かすときには，コントローラの`Y`ボタンを押してから`B`ボタンを押して`auto`モードにしておきましょう．

1メートルほど前に進んだあと，左に90度程度旋回し，右に90度程度旋回したら成功です．



```shell
(開発PC)(docker):~/roomba_hack# rosrun navigation_tutorial simple_control2.py
```

try it! `simple_control2.py`の中身を読んでコードを変更してみよう

{{< /spoiler >}}
