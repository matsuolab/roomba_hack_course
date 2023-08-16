---
title: serviceとactionlib
date: '2023-07-26'
type: book
weight: 20
---

<!--more-->

## Learn

これまでは，トピックを使った通信によりロボットシステムを構築してきました．
トピック通信には，
- 通信の相手を仮定しない ... subscriberがいなくてもpublishできる  
- 非同期通信・処理ができる ... publisherはメッセージをpublishしたらすぐに次の処理を始める（=subscriberの処理が終わるのを待たない）  

などの特徴があり、シンプルにノード間の通信を実現できました．  
しかし，逆に言えば，
- 通信の相手を仮定する ... 呼び出し側のノードが，処理を実行するノードの処理結果によって振るまいを変える
- 同期通信・処理 ... 呼び出し側のノードが，処理を実行するノードで処理が完了するまで待つ

のような処理（クライアント・サーバ型の通信と呼ばれることが多い）を
トピックによって実現することは大変です．

このような比較的複雑な通信を実現するための通信方式として，
ROSはサービス（service）とアクション（actionlib）を用意しています．

### service
#### serviceとは
service通信は，クライアント・サーバ型の通信を実現する最も基本的な方法です。

サービスを提供するノード(service server)と，サービスを呼び出すノード(service client)の間の通信を可能にします．  
以下のような流れで使用します．

1. clientがserverに引数を渡す(処理をリクエストする)
1. 引数を受け取ったserverが何らかの処理を実行する
1. serverが処理結果を返り値としてclientに返す
1. clientは受け取った値に基づいて処理を続行する

{{< figure src="../ros_service.png">}}

pythonでは，rospyモジュールの`rospy.Service()`や
`rospy.ServiceProxy()`を使用することで，サーバ・クライアント
を簡単に実装することができます
（[参考](http://wiki.ros.org/ja/ROS/Tutorials/WritingServiceClient%28python%29)）．

以下のように，コマンドラインからサービスを利用することもできます．
```bash
# サーバを呼び出す
$ rosservice call <ServiceName> <Arguments>

# 存在するサービスの一覧を表示
$ rosservice list

# サービスのメッセージ型を表示
$ rosservice type <ServiceName>
```

#### serviceのデータ型
サービスにおいて使用されるデータの型は.srvファイルに記述されています．
トピックにおいて使用されるデータの型は.msgファイルに記述されていましたが，
サービスの場合は引数と返り値の二つの形式を定義する必要がある点が異なります．  
例としてstd_srvs/SetBoolを示します．引数と返り値の間に`---`を入れて定義します．
```
bool data
---
bool success
string message
```

### actionlib
サービス通信では，クライアントはサーバ側の処理が終わるまで処理を停止するため，
サーバで長い時間がかかるような処理を行う（計算量が大きい，または，移動に時間がかかるなど）場合には，
クライアントの処理が長い間停止してしまうという問題があります．

処理の呼び出し側のプログラムを停止させずに，かつ，
処理の結果や途中経過を受け取って利用できるようにしたものがアクション(actionlib)通信です．

actionlibは5つのトピック通信を組み合わせることで実現されています．
詳しい説明は[qiitaのROS講座](https://qiita.com/srs/items/a39dcd24aaeb03216026#%E6%A6%82%E8%A6%81)を参照してください．

pythonでは，actionlibのサーバやクライアントも，
```python
import actionlib
```
したのちに，他の通信方式と同様に`actionlib.SimpleActionServer`として，簡単に作成できます（[ドキュメント](http://wiki.ros.org/ja/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29)）．

今回の演習では，簡単のためaction serverの作成は行いません．
変わりに，移動のためのactionとして，`move_base`パッケージの中で定義されている`move_base`というactionを使うことにしましょう．
<!-- service serverやservice clientも作成してないが...? -->

実はこのパッケージは
```bash
roslaunch navigation_tutorial navigation.launch
```
を実行して`move_base`ノードを起動した際に既に利用されていました．
<!--（これまでは，そのパッケージの中でサブスクライバとして定義された`move_base_simple/goal`というトピックにpublishすることで移動をしていました）-->

`move_base`パッケージの詳細は[ドキュメント](http://wiki.ros.org/move_base)を参照してください．

同様に，action clientも`actionlib.SimpleActionClient`を利用することで簡単に作成できます．  
例えば，`move_base`のaction clientを実装する際には，
```python
import actionlib
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion

action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
action_client.wait_for_server()  # action serverの準備ができるまで待つ

goal = MoveBaseGoal()  # goalのメッセージを作成
goal.target_pose.header.frame_id = 'map'  # マップ座標系でのゴールとして設定
goal.target_pose.header.stamp = rospy.Time.now()  # 現在時刻
# ゴールの姿勢を指定
goal.target_pose.pose.position.x = X
goal.target_pose.pose.position.y = Y
q = tf.transformations.quaternion_from_euler(0, 0, YAW)  # 回転はquartanionで記述するので変換
goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

action_client.send_goal(goal)  # サーバにゴールを送信
```
のようにクライアントの`send_goal`メソッドでゴールを指定できます．

その後，
```python
action_client.wait_for_result(rospy.Duration(30))
```
とすると，結果が返ってくるまで（この場合30秒間）クライアントの処理をブロックすることができ，
```python
result = action_client.wait_for_result(rospy.Duration(30))
```
とすることで，`result`変数に処理の結果を格納できます．


## 演習

{{< spoiler text="【jetson・開発マシン】起動準備" >}}
```shell
cd roomba_hack
git fetch
git checkout feature/integrate
(jetson) ./RUN-DOCKER-CONTAINER.sh
(開発マシン) ./RUN-DOCKER-CONTAINER.sh 192.168.10.7x
```
{{< /spoiler >}}


{{< spoiler text="【開発マシン】scriptベースのnavigationを実行してみる" >}}
```shell
(開発マシン)(docker) roslaunch navigation_tutorial navigation.launch

(開発マシン)(docker) rosrun navigation_tutorial topic_goal.py
(開発マシン)(docker) rosrun navigation_tutorial action_goal.py
```
{{< /spoiler >}}

{{< spoiler text="【開発マシン】RealSenseで検出した障害物をコストマップに追加してみよう" >}}
```shell
(開発マシン)(docker) roslaunch three-dimensions_tutorial detection_pc.launch
```
{{< /spoiler >}}

{{< spoiler text="（総合課題）障害物を避けながらnavigationする" >}}

Lidarに映らない物体も画像ベースで検出しコストマップに追加することでナビゲーション時にぶつからないようにしましょう。

ヒント
- 物体検出結果に基づいて物体部分以外をマスクしたデプス画像をpublishする
- depth2pc.launchでそれをsubscribeし、point(cloud)に変換する
- 変換されたpointからmap座標系での位置を取得する
- costmapに反映する
- `move_base`アクションを使ってナビゲーションを実装しよう．
  - するとactionがタイムアウトした場合や，`KeyboardInterrupt`された場合に`cancel_goal`メソッドを使うことでactionをキャンセルできるように拡張できるはずです．

さらに，PyTorchを使用した自作の分類器やネット上の分類器をシステムに組み込んで（例えばセグメンテーションモデルなど），よりよく動作するように改良してみましょう．


{{< /spoiler >}}
