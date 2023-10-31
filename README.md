# lidar_smoke_judge

# インストール
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
## テスト用rosbagの再生
[こちら](https://qiita.com/bushio/items/ce0b8fdf42004bada01b)を参考に 評価用の rosbag を再生する。

## 実行
別のターミナルで、docker を起動し、下記を実行する
```
cd /aichallenge/
bash build.sh
source aichallenge_ws/install/setup.bash
ros2 run lidar_smoke_judge lidar_smoke_judge_node 
```