# lidar_smoke_judge

# インストール
## 自動運転AIチャレンジ環境のインストール
[こちら](https://github.com/AutomotiveAIChallenge/aichallenge2023-integ/tree/main)から、自動運転AIチャンレンジ2023のリポジトリをダウンロードする.

```
cd ~/
git clone git@github.com:AutomotiveAIChallenge/aichallenge2023-integ.git
```
ワークスペースのsrcに本リポジトリをダウンロードする.

```
cd ~/aichallenge2023-integ/docker/aichallenge/aichallenge_ws/src/
git clone git@github.com:bushio/lidar_smoke_judge.git
```
## Lidar データ変換用パッケージのインストール
### ros-drivers/velodyne
```
git clone https://github.com/ros-drivers/velodyne
cd velodyne/
git checkout ros2
```
### drive_data_collector
```
git clone https://github.com/bushio/drive_data_collector.git
```


# ビルド
docker を起動する.
```
cd ~/aichallenge2023-integ/docker/
bash build.sh 
bash run_container.sh
```

ビルドを実行する.
```
cd /aichallenge/
bash build.sh
source aichallenge_ws/install/setup.bash
```

# 実行
## Terminal 1 (テスト用rosbagの再生)
[こちら](https://qiita.com/bushio/items/ce0b8fdf42004bada01b)を参考に 評価用の rosbag を再生する。

## Terminal 2 (lidr_data_collectorの実行)
```
cd /aichallenge/
bash build.sh
source aichallenge_ws/install/setup.bash
ros2 launch drive_data_collector lidar_data_collector.launch.xml
```

## Terminal 3 (lidar_smoke_judge の実行)
別のターミナルで、docker を起動し、下記を実行する
```
cd /aichallenge/
bash build.sh
source aichallenge_ws/install/setup.bash
ros2 run lidar_smoke_judge lidar_smoke_judge_node 
```