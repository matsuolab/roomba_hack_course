---
title: システムの構成
date: '2023-08-25'
type: book
weight: 1
---

Roomba Hackにおける
計算機やDockerコンテナのシステム構成について説明します。  
各回の演習でRoombaのセットアップ時に実行するコマンドの意味（目的）を大まかに理解し、
プログラムを実行したい時にどのDockerコンテナに入るべきかがわかるようになる
ことを目指します。

## Lecture

はじめにシステム構成を説明した後、
node同士の通信を可能にするros masterの機能について説明し、
最後に各回の講義で実行するコマンドの意味を説明します。

### システム構成

以下の図が、システムの完成形を表しています。
  
{{< figure src="../system_structure.png">}}

Roombaに載っているJetsonと開発PCのそれぞれで、Dockerコンテナが１つづつ起動しています。
便宜的にコンテナ1、コンテナ2と呼びます。

Jetson側では、Roombaのセンサやアクチュエータ（モータなど、
動力を生み出すもの）を制御するためのノードが動作します。
比較的低レイヤーの動作を担っています。

開発PC側で動作するノードは、Jetson側のノードと比べて高レイヤーの機能を担っています。
開発PCはJetsonと比べスペックが高く、計算量の大きい処理も行うことができます。
各回の演習で編集するソースコードは、開発PC側のDockerコンテナ内で動作するものです。

### ros masterとROS_MASTER_URIの役割

[ROS講座20 複数のPCでROS接続1](https://qiita.com/srs/items/7d4aeb5e44138f97c770)を参照してください。

### コマンドの意味
コンテナ1内のノードとコンテナ2内のノードがデータをやり取りするためには、
同じros masterに問い合わせを行う必要があります。

#### Jetson側で実行するコマンド
各回の演習では、
初めにJetsonにssh接続して
```
~/group_x/roomba_hack$ ./RUN-DOCKER-CONTAINER.sh
```
を実行することでコンテナ1を起動し、
そのままコンテナ1内で
```
~/group_x/roomba_hack$ roslaunch roomba_bringup bringup.launch
```
を実行した時点で、すでにros masterが起動しています。

#### 開発PC側で実行するコマンド
つぎに、開発PCで
```
$ ./RUN-DOCKER-CONTAINER.sh 192.168.10.7x
```
とすることで、コンテナ2を起動しています。
この時、引数にJetsonのIPアドレスを指定することで、
コンテナ1と同じros masterに問い合わせを行っています。

ここまでで、必要な準備は完了しています。  
他のプログラムを新たに実行したい場合には、すでに存在しているコンテナ2内で実行してください。
具体的には、開発PCで新しいターミナルを開き、

```
~/group_x/roomba_hack$ ./RUN-DOCKER-CONTAINER.sh
```
(すでに開発PC上でコンテナが起動している場合には引数はいらない)
を実行することでコンテナ2内に入ることができます。

あるいは、新しいターミナルで
```
$ docker exec -it roomba_hack bash
```
を実行しても同じことができます。

その後、`rosrun`コマンドや`roslaunch`コマンドで、実行したいプログラムを実行してください。
<br>
<br>
<br>
補足  
`$ ./RUN-DOCKER-CONTAINER.sh`や  
`$ roslaunch roomba_bringup bringup.launch`のより詳細な動作が知りたい場合は、
ファイルの中身を読んで確認してください。
