---
title: 自己位置推定
date: '2022-01-22'
type: book
weight: 20
---

<!--more-->

## Learn

前回の演習では、オドメトリを用いてロボットを制御しました。

<!--1m進む、90度右回転など、ある程度正確に動いたかと思います。
しかし、これが数10m前進や、数分間動き続けた時にロボット自身は自分がスタートの時からどのくらい動いたかわかるでしょうか。-->

ルンバが用いているホイールオドメトリは、ホイールの回転量を足し合わせることで算出しています。
長い距離を動かしたり、長時間動かしているとセンサの僅かな誤差の積み重ねで徐々にずれが大きくなってしまいます。

そこで今回は、オドメトリ情報だけでなく、地図とLiDARスキャン情報も同時に使いながら、ロボット自身の尤もらしい位置を推定していきましょう。

### ROSにおける座標系の扱い
{{< spoiler text="座標系を扱うモチベーション">}}
ロボット頭部の距離画像カメラ（デプスカメラ）を用いて、2メートル前にりんごを見つけたとします。  
この時、ロボットのハンドを2メートル前に動かしても、りんごに触れられるとは限りません。  
ロボットには体積があり、各センサやアクチュエータ（ここではカメラとハンド）は距離的に離れているためです。  
センサから得た情報を用いてロボットが何らかの動作を行うには、得たセンサの情報や動作の指令がどの座標系を前提としているのかを意識する必要があり、また、異なる座標系の変換を適切に行う必要があります。
{{</ spoiler>}}

#### tf
[tf](http://wiki.ros.org/tf2)は、

- ロボット座標系
- センサの座標系
- ロボットの関節の座標系
- 部屋の座標系
- 物体の座標系
- 地図の座標系

など多くの座標系同士を繋げ、ロボットシステム上で座標系の管理をしてくれるROSのモジュールです。

```
$ rosrun rqt_tf_tree rqt_tf_tree
```
を実行すると、各座標系の間の関係を可視化することができます。  
tfは、座標系の関係をツリー構造で管理します。親の座標系が複数あることは許されません。

今回自己位置推定を行うにあたり用いる座標系の関係は以下のようになります。

{{< figure src="../rqt_tf_tree.png" caption="tfツリーをrqtで可視化" >}}

ここで、odom座標系は、オドメトリの算出を始めた位置(起動した位置)を原点とした座標系で、
ホイールオドメトリの値から、ロボットの基準となるbase_footprint座標系を繋げています。
base_footprint座標系の下には、ルンバロボットの構成要素であるセンサ類やホイールなどの座標系が子として繋がっています。

一番親にいるmap座標系は、地図の原点を基準とした座標系ですが、
この座標系におけるロボットの座標系(base_footprint)を繋げること、
つまり、ロボットが地図上のどこにいるのかを決めることが、
自己位置推定の目的になります。  
今回の場合、base_footprintの親には既にodomがいるため、
map座標系とodom座標系を繋げることで、全体をひとつのツリーとして管理することができます。


### 自己位置推定

地図が事前に与えられているという前提のもと、
その地図上のどこにロボットがいるのか、
LiDARやOdometryなどのセンサ情報を用いて推定することを自己位置推定といいます。

ここでは、自己位置推定法の一般的な手法の一つであるMCL(Monte Carlo Localization)について説明します。
MCLは、モンテカルロ法（Monte Carlo method）と呼ばれる確率的な手法を用いて、ロボットの位置推定を行います。

以下のような手順で動作します。

1. パーティクルの生成  
  ロボットの位置を表す候補となるパーティクル(粒子)を生成します。
  パーティクルは、ロボットの姿勢（位置と角度）の候補を表す状態ベクトルです。
1. パーティクルの重み付け  
  センサデータやロボットの制御入力（移動量など）との一致の度合いに応じて、各パーティクルに重みを割り当てます。
  センサデータには、LiDARのスキャンデータやカメラの画像データなどが使われます。
1. リサンプリング  
  重み付けされたパーティクルから、新たなパーティクルを生成します。
  重みが大きいパーティクルほど、新しいセットに多く含まれる可能性が高くなります。
  これにより、より確からしい位置候補が残るようになります。
1. パーティクルの移動  
  ロボットが移動すると、パーティクルも同様に移動します。
  パーティクルの移動には、ロボットの制御入力やノイズモデルが使用されます。
  ロボットの制御入力と実際の移動距離には誤差が含まれるため、
  この時パーティクルの分布は広がります。
1. パーティクルの更新  
  ロボットがセンサデータを取得すると、各パーティクルの重みが再計算されます。
  予測されたセンサーデータと実際のセンサーデータの一致度に基づいて重みが更新されます。
1. 推定された位置の計算  
  更新された重みに基づいて、推定されたロボットの位置を計算します。
  一般に、重み付き平均や最も重みの大きいパーティクルの位置が使用されます。

MCLは、センサデータのノイズや環境の不確実性に対して柔軟に対応できるため、自己位置推定の際によく使用されます。
ただし、パーティクル数が十分でない場合や、環境の動きが大きい場合には、正確な位置推定が難しくなる可能性があります。
また、MCLは計算コストが高いため、リアルタイムの応用には制約があります。

MCLのパフォーマンスを向上させるためには、
パーティクル数やリサンプリングの戦略の選択、センサモデルの精度向上など、
いくつかの改良手法があります。

また、他の自己位置推定手法との組み合わせや統合が行われることもあります。

roomba_hackでは、[amcl(Adaptive Monte Carlo Localization)](http://wiki.ros.org/amcl)パッケージと
[emcl(MCL with Expansion resetting)](https://github.com/ryuichiueda/emcl)パッケージが用意されています。

<!--
自己位置推定においては、

- ヒストグラムフィルタ
- カルマンフィルタ
- パーティクルフィルタ

などいくつかの手法が存在し、それぞれメリットデメリットがありますが、
ここでは代表的なパーティクルフィルタを用いた手法を紹介します。

自己位置推定では、観測モデルと状態遷移モデルを交互に繰り返すことによって、
ロボット自身がどこにいるかの確率分布を更新していくことで自己位置推定をしていきます。

パーティクルフィルタでは、この確率分布を大量の粒子を用いて表現する手法になっていて、
各粒子が位置とそこにロボットがいるであろう確率(尤度)を持っています。

ロボットが動くごと(オドメトリが更新されるごと)に、
状態遷移モデルを用いて各粒子の位置情報を更新します。
この時、一般的に分布は広がります。
(人間が目を閉じて歩いたらどこにいるか分かりづらくなるのと同じ)

外界の情報がわかるごと(スキャン情報が更新されるごと)に、
観測モデルを用いて各粒子の尤度を更新します。
尤度は、各粒子の位置から観測できるであろうスキャン情報と、
実際のロボットで取得したスキャン情報との差から算出します。
-->

{{< figure src="../montecarlolocalization.gif" caption="Monte Carlo Localization(Particle Filter) Dieter Fox et al. 1999, using sonar. http://www.doc.ic.ac.uk/~ajd/Robotics/RoboticsResources/montecarlolocalization.gif" >}}

<!-- リサンプリング -->

[詳しい文献]
- "Probabilistic Robotics" by Sebastian Thrun, Wolfram Burgard, Dieter Fox
  - 邦訳："確率ロボティクス" 上田隆一

### launchファイルとrosparam

自己位置推定では、初期位置がどこか、レーザーのスペックや、パーティクルの数など数十個のパラメータを保持します。

これらをプログラム内部で記述するのではなく、launchファイル内で指定することが可能です。
rosでは、rosparamという形でパラメータを管理することが可能です。

以下に、今回用いる`amcl.launch` を示します。
launchファイルはxml形式で記述され、paramを指定すること以外にも、
launchファイル実行時に引数で指定可能なargや、トピック名などのリマップをすることも可能です。

launchの詳しい書き方は、[rosのドキュメント](http://wiki.ros.org/ja/roslaunch/XML)を参照してください。


``` xml
<?xml version="1.0"?>
<launch>
  <arg name="use_map_topic" default="true"/>
  <arg name="odom_topic" default="/odom" />
  <arg name="scan_topic" default="/scan" />

  <node pkg="amcl" type="amcl" name="amcl" output="screen">
    <remap from="scan" to="$(arg scan_topic)"/>    
    <remap from="odom" to="$(arg odom_topic)"/>    
    <param name="use_map_topic" value="$(arg use_map_topic)"/>

    <param name="initial_pose_x" value="0.0"/>
    <param name="initial_pose_y" value="0.0"/>
    <param name="initial_pose_a" value="0.0"/>
    <param name="initial_cov_xx" value="0.1*0.1"/>
    <param name="initial_cov_yy" value="0.1*0.1"/>
    <param name="initial_cov_aa" value="0.3*3.14"/>

    <param name="gui_publish_rate" value="10.0"/>

    <param name="laser_max_beams" value="2.0"/>
    <param name="laser_min_range" value="0.15"/>
    <param name="laser_max_range" value="12.0"/>
    <param name="laser_z_hit" value="0.8"/>
    <param name="laser_z_short" value="0.1"/>
    <param name="laser_z_max" value="0.1"/>
    <param name="laser_z_rand" value="0.1"/>
    <param name="laser_sigma_hit" value="0.2"/>
    <param name="laser_lambda_short" value="0.1"/>
    <param name="laser_model_type" value="likelihood_field"/>
    <param name="laser_likelihood_max_dist" value="2.0"/>

    <param name="min_particles" value="100"/>
    <param name="max_particles" value="1000"/>
    <param name="kld_err" value="0.0"/>
    <param name="kld_z" value="0.0"/>
    <param name="update_min_d" value="0.1"/>
    <param name="update_min_a" value="0.1"/>
    <param name="resample_interval" value="1"/>
    <param name="transform_tolerance" value="0.2"/>
    <param name="recovery_alpha_slow" value="0.001"/>
    <param name="recovery_alpha_fast" value="0.1"/>

    <param name="odom_frame_id" value="odom"/>
    <param name="odom_model_type" value="diff"/>
    <param name="odom_alpha1" value="0.2"/>
    <param name="odom_alpha2" value="0.2"/>
    <param name="odom_alpha3" value="0.2"/>
    <param name="odom_alpha4" value="0.2"/>
    <param name="odom_alpha5" value="0.2"/>

  </node>

</launch>
```

## 演習

{{< spoiler text="【jetson・開発マシン】それぞれdockerコンテナを起動" >}}

jetsonでdockerコンテナを起動
```shell
(開発PC):~$ ssh roomba_dev1
(jetson):~$ cd ~/group_a/roomba_hack
(jetson)::~/group_a/roomba_hack$ git pull 
(jetson):~/group_a/roomba_hack$ ./RUN-DOCKER-CONTAINER.sh
(jetson)(docker):~/roomba_hack# roslaunch roomba_bringup bringup.launch
```

開発PCでdockerコンテナを起動
```shell
(開発PC):~$ cd ~/group_a/roomba_hack
(開発PC):~/group_a/roomba_hack$ git pull
(開発PC):~/group_a/roomba_hack$ ./RUN-DOCKER-CONTAINER.sh 192.168.10.7x
```

{{< /spoiler >}}

{{< spoiler text="gmappingで地図作成" >}}

```
(開発PC)(docker) roslaunch navigation_tutorial gmapping.launch
```

地図の保存。map.pgm（画像データ）とmap.yaml(地図情報)が保存される。
```
(開発PC)(docker) rosrun map_server map_saver
```
`~/roomba_hack/catkin_ws/src/navigation_tutorial/map` の下に保存する。

{{< /spoiler >}}

{{< spoiler text="amclをlaunchして、自己位置推定する" >}}

localizationノードと地図サーバーを同時に起動。
```
(開発PC)(docker) roslaunch navigation_tutorial localization.launch
(開発PC)(docker) roslaunch roomba_teleop teleop.launch
(開発PC)(docker) rviz -d /root/roomba_hack/catkin_ws/src/navigation_tutorial/configs/navigation.rviz
```
- 初期位置の指定(rvizの2D Pose Estimate)
- コントローラで移動させてみて自己位置を確認
- rqt_tf_treeを見てみる

{{< /spoiler >}}

{{< spoiler text="amclのparamをチューニングする" >}}
launchファイルの中身を見てみて、値を変えてみる。

各パラメータの意味は[amclのページ](https://wiki.ros.org/amcl#Parameters)を参照。

例えば、・・・
- initial_cov_** を大きくしてみて、パーティクルがちゃんと収束するかみてみる。
- particleの数(min_particles、max_particles)を変えてみて挙動をみてみる。

{{< /spoiler >}}

{{< spoiler text="launchファイルの拡張" >}}
localization.launchファイルに以下を追加してteleop.launchとrvizが同時に起動するようにしてみよう。
```
<!-- teleop.launchを起動-->
<include file="$(find roomba_teleop)/launch/teleop.launch">
</include>

<!-- rvizを起動-->
<node pkg="rviz" type="rviz" name="navigation_rviz" args="-d $(find navigation_tutorial)/configs/navigation.rviz"/>
```

{{< /spoiler >}}
