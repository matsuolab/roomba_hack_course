---
title: 三次元画像処理
date: '2022-01-22'
type: book
weight: 20
---

<!--more-->

## Learn

<!--今回はRealSenseD435というRGBDカメラを用いて三次元画像処理を行っていきましょう。-->
RGBDカメラを用いて三次元画像処理を行いましょう。

### RGBDカメラについて

RGBDカメラとは、色(RGB)に加え、深度(Depth)情報を取得できるカメラのことです。
周りの環境を三次元の空間として認識することで、ロボットはより複雑にふるまうことができます。
比較的安価でよく利用されるRGBDカメラとして、Intel社製のRealSenseやMicrosoft社製のXtion(エクシオン)などがあります。

### RealSense

今回はRGBDカメラとしてRealSenseD435を使用します。

ROSで用いる際には[標準のラッパー](https://github.com/IntelRealSense/realsense-ros)を使用します。

```
roslaunch realsense2_camera rs_camera.launch
```

を実行すると、2種類のトピック

`/camera/color/image_raw` (RGB画像)  
`/camera/depth/image_raw` (デプス画像)  

が利用できるようになります。  
これらのトピックはいずれも`sensor_msgs/Image`型です。

RealSenseはRGB画像モジュールとデプス画像モジュールが物理的に離れています。
このため、これら2つのトピックはいずれも画像データではあるものの、ピクセルの位置関係が対応しておらず、そのまま画像処理に利用することはできません。

RealSenseを使用するためのlaunchファイル([rs_camera.launch]())を起動する際に
"align_depth"パラメータを"true"に指定することで、デプス画像をRGB画像のピクセルに対応するように変換した
`/camera/aligned_depth_to_color/image_raw`トピックが使用できるようになります。

ただし、roomba_bringupパッケージのbringup.launchファイルの

```
    <include file="$(find realsense2_camera)/launch/rs_camera.launch" if="$(arg realsense)">
        <arg name="align_depth" value="true"/>
    </include>
```

の箇所がこの操作に対応していため、今回は特別な操作をせずとも
`/camera/aligned_depth_to_color/image_raw`トピックを使用できます。

### 物体検出

まずは3次元情報を扱わず、RGB画像`/camera/color/image_raw`のみを用いて画像検出を行ってみましょう。

以下は、
[three-dementions_tutorial](https://github.com/matsuolab/roomba_hack/tree/master/catkin_ws/src/three-dimensions_tutorial)パッケージの
[object_detection.py](https://github.com/matsuolab/roomba_hack/blob/master/catkin_ws/src/three-dimensions_tutorial/scripts/object_detection.py)
です。

1. `/camera/color/image_raw`をsubscribeし、
1. 物体検出アルゴリズムであるYOLOv8に入力し、
1. 物体検出の結果をbounding boxとして描画し、
1. `/detection_result`としてpublish

の処理を行っています。

```py
#!/usr/bin/env python3

import copy
from typing import List

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO
from ultralytics.engine.results import Results


class ObjectDetection:
    def __init__(self):
        rospy.init_node('object_detection', anonymous=True)

        # Publisher
        self.detection_result_pub = rospy.Publisher('/detection_result', Image, queue_size=10)

        # Subscriber
        rospy.Subscriber('/camera/color/image_raw', Image, self.callback_rgb)

        self.bridge = CvBridge()
        self.rgb_image = None

        self.model = YOLO('yolov8n.pt')

    def callback_rgb(self, data):
        cv_array = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.rgb_image = cv_array

    def process(self):
        while not rospy.is_shutdown():
            if self.rgb_image is None:
                continue

            results: List[Results] = self.model.predict(self.rgb_image)

            # plot bounding box
            tmp_image = copy.deepcopy(self.rgb_image)
            for result in results:
                boxes = result.boxes.cpu().numpy()
                names = result.names
                for xyxy, conf, cls in zip(boxes.xyxy, boxes.conf, boxes.cls):
                    if conf < 0.5:
                        continue
                    x1, y1, x2, y2 = map(int, xyxy[:4])
                    cls_pred = cls
                    tmp_image = cv2.rectangle(tmp_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    tmp_image = cv2.putText(tmp_image, names[cls_pred], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

            # publish image
            detection_result = self.bridge.cv2_to_imgmsg(tmp_image, "bgr8")
            self.detection_result_pub.publish(detection_result)


if __name__ == '__main__':
    od = ObjectDetection()
    try:
        od.process()
    except rospy.ROSInitException:
        pass
```

画像データを物体検出モデルに入力として渡すためには、その画像データの型がnp.ndarray型である必要があります。
そのため、コールバック関数で、受け取った`sensor_msgs/Image`型の画像データをnp.ndarray型に変換しています。

```py
cv_array = self.bridge.imgmsg_to_cv2(data, 'bgr8')
```

の部分がこの処理に対応します。  
subscriberを宣言するときにコールバック関数を指定して、
subscribeしたデータをこの関数に渡すという基本的な処理の流れは、
`scan`など他のトピックを扱うsubscriberと同じです。

ここで、YOLOの推論部分をコールバック関数内で行っていないことに注意しましょう。
一見、新しいデータが入ってくるときのみに推論を回すことは合理的に見えますが、
センサの入力に対してコールバック関数内の処理が重いと処理するデータが最新のものからどんどん遅れてしまいます。
コールバック関数はセンサデータの最低限の処理にとどめ、重い処理は分けて書くことを意識しましょう。

また、ここでは既存の物体検出モジュールを使用しましたが、PyTorchなどで自作したモデルも同様の枠組みで利用することができます。

### 三次元画像処理

次に、RGB画像とデプス画像を統合し、検出した物体までの距離を測定してみましょう。

`/camera/color/image_raw` (RGB画像)
`/camera/aligned_depth_to_color/image_raw` (整列されたDepth画像)

はピクセル同士が対応するように処理されてはいるものの、
パブリッシュされた時刻は独立しているため、
併せて使用するには時刻の同期を行う必要があります。

画像の時刻同期には[message_filters](http://wiki.ros.org/message_filters)がよく使われます。

message_filters.ApproximateTimeSynchronizerを使い、以下のようにSubscriberを作成します。

```python
#!/usr/bin/env python3

import copy
from typing import List

import cv2
import message_filters
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class DetectionDistance:
    def __init__(self):
        rospy.init_node('detection_distance', anonymous=True)

        # Publisher
        self.detection_result_pub = rospy.Publisher('/detection_result', Image, queue_size=10)

        # Subscriber
        rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 1.0).registerCallback(self.callback_rgbd)

        self.bridge = CvBridge()
        self.rgb_image, self.depth_image = None, None

    def callback_rgbd(self, data1, data2):
        cv_array = self.bridge.imgmsg_to_cv2(data1, 'bgr8')
        cv_array = cv2.cvtColor(cv_array, cv2.COLOR_BGR2RGB)
        self.rgb_image = cv_array

        cv_array = self.bridge.imgmsg_to_cv2(data2, 'passthrough')
        self.depth_image = cv_array
# 後略
```

この例では、  
`/camera/color/image_raw`と  
`/camera/aligned_depth_to_color/image_raw`の  
トピックを同時(1.0秒までのずれを許容する)に受け取った場合のみ、
2つの画像データをコールバック関数callback_rgbdに渡します。

それでは、[three-dementions_tutorial](https://github.com/matsuolab/roomba_hack/tree/master/catkin_ws/src/three-dimensions_tutorial)
パッケージの[detection_distance.py](https://github.com/matsuolab/roomba_hack/blob/master/catkin_ws/src/three-dimensions_tutorial/scripts/detection_distance.py)
を見てみましょう。  
物体を検出し、その物体までの距離を測定するスクリプトです。

```py
#!/usr/bin/env python3

import copy
from typing import List

import cv2
import message_filters
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO
from ultralytics.engine.results import Results


class DetectionDistance:
    def __init__(self):
        rospy.init_node('detection_distance', anonymous=True)

        # Publisher
        self.detection_result_pub = rospy.Publisher('/detection_result', Image, queue_size=10)

        # Subscriber
        rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 1.0).registerCallback(self.callback_rgbd)

        self.bridge = CvBridge()
        self.rgb_image, self.depth_image = None, None

        self.model = YOLO('yolov8n.pt')

    def callback_rgbd(self, data1, data2):
        cv_array = self.bridge.imgmsg_to_cv2(data1, 'bgr8')
        cv_array = cv2.cvtColor(cv_array, cv2.COLOR_BGR2RGB)
        self.rgb_image = cv_array

        cv_array = self.bridge.imgmsg_to_cv2(data2, 'passthrough')
        self.depth_image = cv_array

    def process(self):
        while not rospy.is_shutdown():
            if self.rgb_image is None:
                continue

            # inference
            tmp_image = copy.copy(self.rgb_image)

            results: List[Results] = self.model.predict(self.rgb_image, verbose=False)

            # plot bouding box
            for result in results:
                boxes = result.boxes.cpu().numpy()
                names = result.names
                if len(boxes.xyxy) == 0:
                    continue
                x1, y1, x2, y2 = map(int, boxes.xyxy[0][:4])
                cls_pred = boxes.cls[0]
                tmp_image = cv2.rectangle(tmp_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                tmp_image = cv2.putText(tmp_image, names[cls_pred], (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cx, cy = (x1+x2)//2, (y1+y2)//2
                print(names[cls_pred], self.depth_image[cy][cx]/1000, "m")

            # publish image
            tmp_image = cv2.cvtColor(tmp_image, cv2.COLOR_RGB2BGR)
            detection_result = self.bridge.cv2_to_imgmsg(tmp_image, "bgr8")
            self.detection_result_pub.publish(detection_result)


if __name__ == '__main__':
    dd = DetectionDistance()
    try:
        dd.process()
    except rospy.ROSInitException:
        pass
```

基本的には物体検出のスクリプトと同じですが、

```py
cx, cy = (x1+x2)//2, (y1+y2)//2
print(names[cls_pred], self.depth_image[cy][cx]/1000, "m")
```

でbounding boxの中心座標を変換し、対応する距離をメートル単位で表示しています。

整列されたデプス画像を用いているため、RGB画像に基づき算出した座標をそのまま指定できます。

### 点群の作成

上の例ではRGB画像とDepth画像を用いて、物体位置の代表となる点とロボット位置の関係を扱うことができました。

しかし、代表となる点以外の深度データも三次元空間に直接マッピングできると、
物体の大きさや形状といった情報も扱うことができ、
環境情報をより直感的・統一的に扱うことができるように思われます。

そこでDepth画像から点群と呼ばれるデータを作成することを考えます。

点群とは三次元座標値 (X, Y, Z) で構成された点の集まりのことです。各点の情報として、三次元座標値に加え色の情報 (R, G, B) が加わることもあります。
デプス画像はカメラの内部パラメータを用いることによって点群データに変換することができます。([参考](https://medium.com/yodayoda/from-depth-map-to-point-cloud-7473721d3f))

今回は、デプス画像を点群データに変換するためのROSの外部パッケージである
[depth_image_proc](http://wiki.ros.org/depth_image_proc)
を使用して点群を作成します。

外部パッケージは`~/catkin_ws/src`等のワークスペースに配置し、ビルドしパスを通すことで簡単に使用できます。

depth_image_procのwikiを参考に以下のようなlaunchファイルを作成しました。

```xml
<?xml version="1.0"?>
<launch>
  <node pkg="nodelet" type="nodelet" name="nodelet_manager" args="manager" />

  <node pkg="nodelet" type="nodelet" name="nodelet1"
        args="load depth_image_proc/point_cloud_xyz nodelet_manager">
    <remap from="camera_info" to="/camera/color/camera_info"/>
    <remap from="image_rect" to="/camera/aligned_depth_to_color/image_raw"/>
    <remap from="points" to="/camera/depth/points"/>
  </node>
</launch>
```

このlaunchファイルを実行すると  
`/camera/color/camera_info`と  
`/camera/aligned_depth_to_color/image_raw`を  
subscribeし、  
`/camera/depth/points`をpublishするノードが作成されます。

`/camera/color/camera_info`は
[sensor_msgs/CameraInfo](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)型のトピックです。
カメラパラメータやフレームid、タイムスタンプなどの情報を保持しており、点群の変換に利用されます。

`/camera/aligned_depth_to_color/image_raw`はRGB画像に合わせて整列されたDepth画像であるため、
`/camera/{{< hl >}}depth{{</ hl >}}/camera_info`ではなく  
`/camera/{{< hl >}}color{{</ hl >}}/camera_info`を指定しています。  

```bash
(開発PC)(docker)# roslaunch three-dimensions_tutorial depth2pc.launch
```

を実行し、`/camera/depth/points`トピックをrvizで可視化をすると三次元空間に点群データが表示されているのが確認できます。

## 演習

{{< spoiler text="(開発PC, jetson)起動準備" >}}

```bash
(jetson)$ ./RUN-DOCKER-CONTAINER.sh
(jetson)(docker)# roslaunch roomba_bringup bringup.launch
(開発PC)$ ./RUN-DOCKER-CONTAINER.sh 192.168.10.7x
```

{{< /spoiler >}}

{{< spoiler text="(開発PC)RealSenseのトピックの可視化" >}}

```bash
(開発PC)(docker)# rviz
```

rviz上で

- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/aligned_depth_to_color/image_raw`  

を可視化して違いを確認してみましょう。
{{< /spoiler >}}

{{< spoiler text="(開発PC)物体検出を行う" >}}

```bash
(開発PC)(docker)# cd catkin_ws; catkin_make; source devel/setup.bash
(開発PC)(docker)# rosrun three-dimensions_tutorial object_detection.py
# rvizで`/detection_result`を表示し結果を確認してみよう。
(開発PC)(docker)# rosrun three-dimensions_tutorial detection_distance.py
```

{{< /spoiler >}}

{{< spoiler text="(開発PC)外部パッケージを使用" >}}

```bash
(開発PC)(docker)# cd ~/external_catkin_ws/src 
(開発PC)(docker)# git clone https://github.com/ros-perception/image_pipeline
(開発PC)(docker)# cd ../; catkin build; source devel/setup.bash
(開発PC)(docker)# cd ~/roomba_hack/catkin_ws; source devel/setup.bash
(開発PC)(docker)# roslaunch three-dimensions_tutorial depth2pc.launch
(開発PC)(docker)# roslaunch navigation_tutorial navigation.launch
```

rvizで`/camera/depth/points`トピックを追加して確認してみましょう。
{{< /spoiler >}}

{{< spoiler text="余裕がある人向け" >}}
物体を検出し、特定の物体の手前まで移動するスクリプトを作ってみましょう。

ヒント

- 物体検出結果に基づいて物体部分以外をマスクしたデプス画像をpublishする
- depth2pc.launchでそれをsubscribeし、point(cloud)に変換する
- 変換されたpointからmap座標系での位置を取得する
- navigation_tutorial/scripts/set_goal.py (map座標系で指定した位置・姿勢までナビゲーションするスクリプト)などを参考に、その位置へとナビゲーションする

PyTorchを使用した自作の分類器やネット上の分類器をシステムに組み込んでみましょう。

Lidarに映らない物体も画像ベースで検出しコストマップに追加することでナビゲーション時にぶつからないようにしましょう。
{{< /spoiler >}}
