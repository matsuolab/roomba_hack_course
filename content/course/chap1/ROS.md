---
title: ROSとは
date: '2022-05-11'
type: book
weight: 21
---

ロボット開発によく用いられるROSの概要を理解する
<!--more-->

## Learn

### ROSの概要
ROS(Robot Operating System)は、ロボット・アプリケーション開発に便利な機能を提供するフレームワークです。
フレームワークとは、プログラミング言語を特定の目的に特化させて使うためのツールのことです．
具体的には以下にあげるものをROSは提供しています。

- メッセージ通信

    プロセス間、コンピュータ間の通信ライブラリが提供されています。  

- デバイスドライバ
    
    ROSに対応しているセンサやアクチュエータを搭載したロボットであれば、違うロボットであってもほぼ同じソースコードを使用して動かすことができます．  
    roombaを動かすために書いたソースコードをそのまま使用してHSRを動かす、といったことができます．  

    https://github.com/ros-drivers  
    http://wiki.ros.org/Sensors

- ライブラリ
    
    ロボットを動作させるソフトウェア(ナビゲーション、マニピュレーション)の基本機能の大半が提供されています。

- **視覚化ツール**

    ロボットの内部状態やセンサ出力を2次元、3次元で視覚化するRvizや3次元動力学シミュレータのGazeboなどが提供されています。

- パッケージ管理

    多種多様なプログラミング言語(python, C++, ...)、依存関係で記述されたプログラム同士を統合的に使用することが可能です。  
    これにより、経路計画など処理が重いプロセスはC++でコードを書き、画像認識など機械学習系のプロセスはpythonでコードを書く、といったこともできるようになります．

### ROSのメッセージ通信
ロボットを動かす際には、多くのプログラムを並列して実行し、それぞれがデータをやりとりする必要があります。
ROSはそのようなプログラム間の通信に必要な機能を提供しています。

- **ノード(node)**

    ROSでは，一つのプログラム単位を「ノード(node)」と呼びます．  
    基本的には，一つのファイルが一つのノードに対応しています．  
    各ノードは次に述べるtopic，service，actionの三つの通信方法を使って，他のノードとデータのやり取りを行います．

- **トピック(topic)**

    ROSでの、最も基本的なデータ通信の経路を「トピック(topic)」と呼びます。
    ノードはメッセージをトピックへ向けて配信(Publish)し、また購読する(Subscribe)ことで他のノードと情報を共有することができます。
    配信を行うノードをPublisher，購読を行うノードをSubscriberと呼びます．ノードはこの二つ必ず分けることができるというわけではなく，実際には一つのノードがpublisherであり、subscriberでもあるという状況がほとんどです．  

    トピックには名前が付けられており、同じトピックに複数のノードがデータを送ったり、複数のノードが同じデータを受け取ることができます。  

    - メッセージ(message)

        トピックへ配信したり、購読したりするときのROSのデータ型のことを「メッセージ(message)」と呼びます。
        メッセージの型はmsgファイルに記述されており、使用するプログラミング言語に依存しないデータ形式になっています。

        以下に、物体やロボットの位置を表す時によく用いる`geomemtry_msgs/PoseStamped`型のmsgファイルを示します。
        位置情報の時間や座標フレームの情報が含まれるheaderと座標位置を表すposeで定義されています。
        ```
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
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
        ```

    {{< figure src="../illust_node.png" caption="Topic通信" >}}

- **サービス(service)**

    「サービス(service)」も，ノードが他のノードと通信するための手段の一つです。少しだけtopicより複雑な通信の仕方を提供します. 

    サービスには、サービスを提供するノード（service server）とサービスを要求するノード(service client)があります．  
    サービスは以下のような流れで使用されます．

    1. service clientがservice serverに引数を渡す．
    1. 引数を受け取ったservice serverが何らかのプログラムを実行する．
    1. service serverは行為の結果を戻り値としてservice clientに返す．
    1. service clientはその戻り値に応じて後の挙動を変える．

    サービスにおいて送受信されるデータの型は.srvファイルに記述されています。
    メッセージと同様使用言語に依存しないデータ形式ですが、メッセージと異なるのは、引数と戻り値の二つの形式を定義する必要があるところです。

    以下に、srvの例として`std_srvs/SetBool`を示します。
    このように引数と戻り値の間に`---`を入れて定義します。
    ```
    bool data
    ---
    bool success
    string message
    ```

- **アクション(action)**

    アクションもノード間通信の一つの手段です．serviceよりもさらに複雑な通信ができます．  
    トピックやサービスほど頻繁には使われないので，ここでは説明を省略します．

- **ROSマスタ(ROS master)**

    「ROSマスタ(ROS master)」は、ノード、トピックおよびサービスの名前登録を行い、それぞれのノードが他のノードから見えるようにする役割を担っています。
    通信するノード名とトピック名およびサービス名の対応が決定した後、ノード同士が「peer-to-peer」で通信します。

    ROSマスタを起動するには「roscore」というコマンドを実行します（が，RoombaやHSRをつかうときにはこのコマンドが自動で実行されることが多いため，あまり意識する機会はないかもしれません）。

<!--
パラメータサーバは必須ではないと思うのでコメントアウト。

- パラメータサーバ(parameter server)

    「パラメータサーバ(parameter server)」は、設定データを複数のノードで共有するための軽量なサーバです。
    各ノードのパラメータを、パラメータサーバで一括して管理できます。
    パラメータサーバもROSマスタ同様に「roscore」コマンドで起動します。

    パラメータサーバで扱える型は、整数・小数・真偽値・辞書・リストになります。
-->

- ROSのデータ通信のまとめ

    {{< figure src="../ros_communication.png" caption="ROS通信" >}}

<!-- ### デバイスドライバ

- カメラ
- LiDAR
- IMU -->

### ROSと連動するソフトウェア
ROSは以下のようなソフトウェアと連動して使うためのパッケージを提供しています。  
簡単な説明にとどめるので，詳しい使い方は必要になったときに勉強してください。  

- **OpenCV**
    
    豊富な機能を持つ2D画像処理用のライブラリです．
    カメラで撮影した画像を処理する際に使用します．

    <!--
    OpenCVのデータ形式である、MatクラスとROSのメッセージ形式を変換するcv_bridgeや３次元座標上の物体を２次元画像上に投影する機能であるimage_geometryといったパッケージ(vision_opencv)が提供されています。
    -->

- **PCL(Point Cloud Library)**

    3次元点群処理のライブラリ．  
    HSRやRoombaにはRGBDカメラが搭載されています．DはDepthという意味で、画像の各ピクセルに距離情報を対応させたDepth画像を取得することができます．  
    このような三次元の点群の情報を処理する際にPCLを使うと便利です．  
    
    <!--OpenCV同様PCLのデータ形式とROSのメッセージ形式を変換するパッケージが提供されています。-->

- **OpenSLAM**

    地図を効果的に使うことで，より安定したロボットのナビゲーションを行うことができます．  
    移動ロボットの自己位置推定と地図生成を同時に行うSLAM(Simultaneous Localization and Mapping)という手法は、それだけで一つの研究分野になる程奥深い分野で，活発に研究が行われています．  
    OpenSLAMは，SLAMのソースコードを公開するためのプラットフォームを提供しており，様々なSLAMの手法を実装しています．

<!--
roombaには手がついていないので，MoveIt!の説明は省略．
- **Move it**
-->

これ以外にも本当にたくさんのツールがROSと連動しています．詳しく知りたい場合は、本やネットで調べてみてください．

### 可視化ツール

ロボット内部の大量のデータが正しく処理されているか知りたい場合，変数の中身の数値などを直接みるのは大変です．直感的にわかりづらいためミスも増えます．  
可視化をすることで，開発やデバッグがより効率よく進められます．

- **rqt**

  rqtはROSのGUIフレームワークで，様々なツールを提供しています．  
  ノードの状態を可視化する`rqt_graph`(下図1)，メッセージの値を時系列に沿ってプロットする`rqt_plot`(下図2)などがあります．

  {{< figure src="../rqt_graph.png" caption="図1 rqt_graph [wikipedia](http://wiki.ros.org/rqt_graph)より引用" >}}

  {{< figure src="../rqt_plot.png" caption="図2 rqt_plot [wikipedia](http://wiki.ros.org/rqt_plot)より引用">}}

 <!-- http://wiki.ros.org/rqt -->

- **RViz**
  
  ロボットの三次元モデルや座標系，測定した三次元点群などを可視化するツールです．  
  三次元空間の情報以外に，カメラに写っている画像なども表示できます．

  {{< youtube i--Sd4xH9ZE >}}
  <!-- http://wiki.ros.org/ja/rviz -->

- **gazebo**

  オープンソースのロボット用三次元動力学シミュレータ．  
  説明は割愛します．  

<!-- ### パッケージ管理
- プログラミング言語
- rosdep
-  -->

## 演習
{{< spoiler text= **roombaドライバを起動し、動作していることを確認する** >}}

- jetsonにアクセスする
    ``` sh
    (開発PC):~$ ssh roomba_dev1
    (jetson):~$
    ```

- docker containerを起動する  
  余裕があれば`RUN-DOCKER-CONTAINER.sh`ファイルの中身を確認してみましょう。
    ``` sh
    (jetson):~$ cd ~/team_a/roomba_hack
    (jetson):~/team_a/roomba_hack$ ./RUN-DOCKER-CONTAINER.sh
    root@roomba-dev-jetson:~/roomba_hack#
    ```
  `root@roomba-dev-jetson:~/roomba_hack#`などと表示されればdocker内部に入れています。
  
  今後docker内部であることは(docker)と表記します。

- roomba driverなどを起動するlaunchファイルを起動する  
  このタイミングでルンバの電源が入っているかを確認しておきましょう。
    ``` sh
    (jetson)(docker):~/roomba_hack# roslaunch roomba_bringup bringup.launch
    ```
  起動に成功すればルンバからピッと短い音が鳴り、ターミナルには赤い文字が出続けるはずです。

{{< /spoiler >}}


{{< spoiler text= **コントローラーを使ってロボットを動かす** >}}

- 開発PCでdocker containerを起動する  
  xにはroomba_devの後につく数字を入れてください。

    ``` sh
    (開発PC):~$ cd ~/team_a/roomba_hack
    (開発PC):~/team_a/roomba_hack$ ./RUN-DOCKER-CONTAINER.sh 192.168.10.7x
    ```
    
- コントローラーを起動  
  コントローラーが開発PCに刺さってることを確認してください。
    ``` sh
    (開発PC)(docker):~/roomba_hack# cd catkin_ws
    (開発PC)(docker):~/roomba_hack/catkin_ws# catkin_make
    (開発PC)(docker):~/roomba_hack/catkin_ws# source devel/setup.bash
    (開発PC)(docker):~/roomba_hack/catkin_ws#roslaunch roomba_teleop teleop.launch
    ```

- コントローラのモード
    - 移動・停止 
    - 自動・マニュアル
    - ドッキング・アンドッキング

- コントローラによる操縦
    - 移動ロック解除
        L1を押している時のみ移動コマンドが動作します。
    - 左ジョイスティック
        縦方向で前進速度(手前に倒すとバック)、横方向は回転速度に対応しています。
    - 左矢印
        それぞれ、一定に低速度で前進・後退・回転します。

- 正常に起動できているかを確認  
  開発PCで新しくターミナルを開いてdockerの中に入ります。
  
  すでに開発PCで起動されているdockerコンテナに入る場合は、
  
  ``` sh
  (開発PC):~/group_a/roomba_hack$ docker exec -it roomba_hack bash
  ```
  または
  ``` sh
  (開発PC):~/group_a/roomba_hack$ ./RUN-DOCKER-CONTAINER.sh
  ```
  のいずれかの方法で入ることができます。
   
  さまざまなコマンドを使ってroombaの情報を取得してみましょう。
    ``` sh
    (開発PC)(docker):~/roomba_hack# rosnode list
    (開発PC)(docker):~/roomba_hack# rostopic list
    (開発PC)(docker):~/roomba_hack# rostopic echo /cmd_vel
    (開発PC)(docker):~/roomba_hack# rqt_graph
    (開発PC)(docker):~/roomba_hack# rviz
    ```

{{< /spoiler >}}
