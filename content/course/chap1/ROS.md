---
title: ROSとは
date: '2023-06-06'
type: book
weight: 21
---

ロボット開発によく用いられるROSの概要を理解する
<!--more-->

## Learn

### ROSの概要
ROS(Robot Operating System)は、ロボット・アプリケーション開発に便利な機能を提供するフレームワークです。
フレームワークとは、プログラミング言語を特定の目的に特化させて使うためのツールのことです。
具体的には以下にあげる機能を提供しています。

- メッセージ通信

    - プロセス間、コンピュータ間の通信ライブラリが提供されています。  

- デバイスドライバ
    
    - ROSに対応しているセンサやアクチュエータを搭載したロボットであれば、違うロボットであってもほぼ同じソースコードを使用して動かすことができます。  
      roombaを動かすために書いたソースコードをそのまま使用してHSRを動かす、といったことができます。  

    - https://github.com/ros-drivers  
    - http://wiki.ros.org/Sensors

- ライブラリ
    
    - ロボットを動作させるソフトウェア(ナビゲーション、マニピュレーション)の基本機能の大半が提供されています。

- 視覚化ツール

    - ロボットの内部状態やセンサ出力を2次元、3次元で視覚化するRvizや3次元動力学シミュレータのGazeboなどが提供されています。

- パッケージ管理

    - 多種多様なプログラミング言語(python, C++, ...)、依存関係で記述されたプログラム同士を統合的に使用することが可能です。  
    - これにより、経路計画など処理が重いプロセスはC++でコードを書き、画像認識など機械学習系のプロセスはpythonでコードを書く、といったこともできるようになります。

### ROSのメッセージ通信
ロボットを動かす際には、多くのプログラムを並列して実行し、それぞれがデータをやりとりする必要があります。
ROSはそのようなプログラム間の通信に必要な機能を提供しています。

- ノード(node)

    - ROSでは、一つのプログラム単位を「ノード(node)」と呼びます。  
    - 基本的には、一つのファイルが一つのノードに対応しています。  
    - 各ノードは次に述べるtopic、service、actionlibの三つの通信方法を使って、他のノードとデータのやり取りを行います。

- トピック(topic)

    - ROSでの、最も基本的なデータ通信の経路を「トピック(topic)」と呼びます。
    - ノードはメッセージをトピックへ向けて配信(Publish)し、また購読する(Subscribe)ことで他のノードと情報を共有することができます。
    - 配信を行うノードをPublisher、購読を行うノードをSubscriberと呼びます。ノードはこのどちらかに二分することができるというわけではなく、実際には一つのノードがpublisherであり、subscriberでもあるという状況がほとんどです。  

    - トピックには名前が付けられており、同じトピックに複数のノードがデータを送ったり、複数のノードが同じデータを受け取ることができます。  

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
          各行の左側にはデータ型が、右側には変数名が記述されています。

    - Topic通信のイメージ
      {{< figure src="../node.png">}}

- サービス(service)

    - 「サービス(service)」も、ノードが他のノードと通信するための手段の一つです。topicより少し複雑な通信の仕方を提供します. 
    - サービスには、サービスを提供するノード（service server）とサービスを要求するノード(service client)があります。  
    - サービスは以下のような流れで使用されます。

      1. clientがserverに引数を渡す。
      1. 引数を受け取ったserverが何らかのプログラムを実行する。
      1. serverは行為の結果を返り値としてclientに返す。
      1. clientはその返り値に応じて後の挙動を変える。

      {{< figure src="../ros_service.png">}}
    
    - サービスにおいて送受信されるデータの型は.srvファイルに記述されています。
    - メッセージと同様使用言語に依存しないデータ形式ですが、メッセージと異なるのは、引数と戻り値の二つの形式を定義する必要があるところです。

    - 以下に、srvの例として`std_srvs/SetBool`を示します。  
      このように引数と戻り値の間に`---`を入れて定義します。
      ```
      bool data
      ---
      bool success
      string message
      ```
      

- アクション(actionlib)

    - アクションもノード間通信の一つの手段です。serviceよりもさらに複雑な通信ができます。  
    - サービスは動作の終了時にのみserverからclientに結果を返すのに対し、アクションは動作の途中経過をclientに渡すことができます。
    - ここでは詳しい説明を省略します。

- ROSマスタ(ROS master)

    - 「ROSマスタ(ROS master)」は、ノード、トピックおよびサービスの名前登録を行い、それぞれのノードが他のノードから見えるようにする役割を担っています。
    - 通信するノード名とトピック名およびサービス名の対応が決定した後、ノード同士が「peer-to-peer」で通信します。

    - ROSマスタを起動するには「roscore」というコマンドを実行します（が、RoombaやHSRをつかうときにはこのコマンドが自動で実行されることが多いため、あまり意識する機会はないかもしれません）。

<!--
パラメータサーバは必須ではないと思うのでコメントアウト。

- パラメータサーバ(parameter server)

    「パラメータサーバ(parameter server)」は、設定データを複数のノードで共有するための軽量なサーバです。
    各ノードのパラメータを、パラメータサーバで一括して管理できます。
    パラメータサーバもROSマスタ同様に「roscore」コマンドで起動します。

    パラメータサーバで扱える型は、整数・小数・真偽値・辞書・リストになります。
-->

### ROSと連動するソフトウェア
ROSは以下のようなソフトウェアと連動して使うためのパッケージを提供しています。簡単な説明にとどめるので、詳しい使い方は必要になった際に調べてください。  

- OpenCV
    
    豊富な機能を持つ2D画像処理用のライブラリです。
    カメラで撮影した画像を処理する際に使用します。

- PCL(Point Cloud Library)

    - 3次元点群処理のライブラリ。  
    - HSRやRoombaにはRGBDカメラが搭載されています。DはDepthという意味で、画像の各ピクセルに距離情報を対応させたDepth画像を取得することができます。  
    このような三次元の点群の情報を処理する際にPCLを使うと便利です。  
    
    <!--OpenCV同様PCLのデータ形式とROSのメッセージ形式を変換するパッケージが提供されています。-->

- OpenSLAM

    - 地図を効果的に使うことで、より安定したロボットのナビゲーションを行うことができます。  
    - 移動ロボットの自己位置推定と地図生成を同時に行うSLAM(Simultaneous Localization and Mapping)という手法は、それだけで一つの研究分野になる程奥深い分野で、活発に研究が行われています。  
    - OpenSLAMは、SLAMのソースコードを公開するためのプラットフォームを提供しており、様々なSLAMの手法を実装しています。

<!--
roombaには手がついていないので、MoveIt!の説明は省略。
- **Move it**
-->

これ以外にも多くのツールがROSと連動しています。

### 可視化ツール

ロボット内部の大量のデータが正しく処理されているか知りたい場合、変数の中身の数値などを直接みるのは大変です。直感的にわかりづらいためミスも増えます。  
可視化をすることで、開発やデバッグがより効率よく進められます。

- rqt

  rqtはROSのGUIフレームワークで、様々なツールを提供しています。  
  ノードの状態を可視化する`rqt_graph`(下図1)、メッセージの値を時系列に沿ってプロットする`rqt_plot`(下図2)などがあります。

  {{< figure src="../rqt_graph.png" caption="図1 rqt_graph [wikipedia](http://wiki.ros.org/rqt_graph)より引用" >}}

  {{< figure src="../rqt_plot.png" caption="図2 rqt_plot [wikipedia](http://wiki.ros.org/rqt_plot)より引用">}}

 <!-- http://wiki.ros.org/rqt -->

- RViz
  
  ロボットの三次元モデルや座標系、測定した三次元点群などを可視化するツールです。  
  三次元空間の情報以外に、カメラに写っている画像なども表示できます。

  {{< youtube i--Sd4xH9ZE >}}
  <!-- http://wiki.ros.org/ja/rviz -->

- gazebo

  オープンソースのロボット用三次元動力学シミュレータ。  
  説明は割愛します。  

<!-- ### パッケージ管理
- プログラミング言語
- rosdep
-  -->

## 演習

{{< spoiler text= roombaドライバを起動し、動作することを確認する >}}

1. jetsonにアクセスする
    ``` sh
    (開発PC):~$ ssh roomba_dev1
    (jetson):~$
    ```

1. dockerコンテナを起動する  
    <!--余裕があれば`RUN-DOCKER-CONTAINER.sh`ファイルの中身を確認してみましょう。-->
    ``` sh
    (jetson):~$ cd ~/23_group_x/roomba_hack

    (jetson):~/23_group_x/roomba_hack$ ./RUN-DOCKER-CONTAINER.sh
    # このファイルを実行することでdockerコンテナを作成し、コンテナの中に入る。

    root@roomba-dev-jetson:~/roomba_hack#
    # 上のように表示されればコンテナ内部に入れています。
    ```
  
    今後docker内部であることは(docker)と表記します。

1. roomba driverなどを起動するlaunchファイルを実行する  
    このタイミングでルンバの電源が入っているかを確認しておきましょう。
    ``` sh
    (jetson)(docker):~/roomba_hack# roslaunch roomba_bringup bringup.launch
    ```
    起動に成功すればルンバからピッと短い音が鳴り、ターミナルには赤い文字が出続けるはずです。

{{< /spoiler >}}

{{< spoiler text= 開発PCでdockerコンテナを起動する >}}
1. 開発PCでdockerコンテナを起動する  

    ```sh
    (開発PC):~$ cd ~/23_group_x/roomba_hack

    (開発PC):~/23_group_x/roomba_hack$ ./RUN-DOCKER-CONTAINER.sh 192.168.10.7x
    # xにはroomba_devの後につく数字を入れてください。
    ```
    - 先ほどjetson内でdockerコンテナを起動しましたが、今回は開発PC内でコンテナを起動します。  
    - このとき引数にjetsonのIPアドレスを入れることで、jetson内のROSマスタ(前述)に、開発PCからアクセスできるようにしています。

1. パッケージのビルド

    ```sh
    (開発PC)(docker):~/roomba_hack# cd catkin_ws
    
    (開発PC)(docker):~/roomba_hack/catkin_ws# ls
    # catkin_ws内に存在するディレクトリを確認する。

    (開発PC)(docker):~/roomba_hack/catkin_ws# catkin_make
    # いろいろな出力が生成される。

    (開発PC)(docker):~/roomba_hack/catkin_ws# ls
    # 再度catkin_ws内に存在するディレクトリを確認する。
    ```

    ここで、buildとdevelというディレクトリが生成されていると、うまくいっています。  
    - build  
      C++のコードを用いる際に、コンパイルされたファイルが生成されるディレクトリ。pythonを使っているときにはほとんど意識しない。
    - devel  
      様々なファイルを含んでいるが、特にsetupファイルが重要。  
      このファイルを実行することで、現在いるワークスペースに含まれるコードを使用するようにROSの環境が設定される。  

1. setupファイルを実行する

    ```sh
    (開発PC)(docker):~/roomba_hack/catkin_ws# source devel/setup.bash
    # setupファイルを実行
    ```

{{< /spoiler >}}

{{< spoiler text= コントローラーを使ってロボットを動かす >}}

- コントローラーを起動  
  コントローラーが開発PCに接続されていることを確認してください。
    ``` sh
    (開発PC)(docker):~/roomba_hack/catkin_ws# roslaunch roomba_teleop teleop.launch
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

{{< /spoiler >}}


{{< spoiler text= Roombaの情報を取得する >}}

1. 開発PCの新しいターミナルでdockerコンテナに入る  

    bringup.launch及びteleop.launchを実行したターミナルは実行中のプログラムに占領されているので、開発PCで新しくターミナルを開いてコンテナの中に入ります。  
    すでに開発PCで起動されているコンテナに入る場合は、
    
    ```sh
    (開発PC):~/23_group_x/roomba_hack$ docker exec -it roomba_hack bash
    # docker exec -it <コンテナ名> bash で起動中のコンテナに入ることができる。
    ```
    または
    ```sh
    (開発PC):~/23_group_x/roomba_hack$ ./RUN-DOCKER-CONTAINER.sh
    ```
    のいずれかのコマンドで入ることができます。
   
1. Roombaの情報を取得する  

    さまざまなコマンドを使ってRoombaの情報を取得してみましょう。  
    ```sh
    (開発PC)(docker):~/roomba_hack# rosnode list
    # ノードの一覧を表示する

    (開発PC)(docker):~/roomba_hack# rostopic list
    # トピックの一覧を表示する

    (開発PC)(docker):~/roomba_hack# rostopic echo /cmd_vel
    # /cmd_velというトピックの中身を表示する
    # teleop.launchを実行している状態でコントローラーを操作すると、/cmd_velの中身が変化することがわかる。

    (開発PC)(docker):~/roomba_hack# rqt_graph
    # ノードとトピックの関係を表示

    (開発PC)(docker):~/roomba_hack# rviz
    # rvizを起動
    ```

{{< /spoiler >}}

{{< spoiler text= プロセスの終了・dockerコンテナから出る >}}

  1. プロセスの終了  
      一部のプログラムは終了するまで処理を続けるため、明示的に終了させる必要があります。

      多くのプログラムは`Ctrl+C`で終了します。

  1. dockerコンテナ・ターミナルから出る

      コマンドライン上で

      ```shell
      exit
      ```

      を実行することで、

      - コンテナの中にいる場合はコンテナ外にでる。
      - sshしている場合はsshを終了する。
      - ターミナルを使用している場合はターミナルを終了する。

      ことができます。また、`Ctrl+D`でも同様のことができます。

{{< /spoiler >}}

