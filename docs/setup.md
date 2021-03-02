## インストール手順

本ツールである Fork 版 Foxy は各環境でビルドする必要があります。

基本的なインストール方法は [Building ROS 2 on Linux ](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/)と同じになります。  
ここでは、 Ubuntu Focal Fossa (20.04) 64-bit を想定します。

* TOC
{:toc}

### システムのセットアップ

#### ビルド関連パッケージのインストール

[System setup](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/#system-setup) にならい、foxy をビルドに必要な環境をセットアップします。  
以下の章のコマンドを実行してください。

1. Set locale
2. Install development tools and ROS tools

#### LTTng のインストール

ROS2 レイヤーのトレースポイントも利用するため、  
事前に LTTng のインストールが必要です。

インストール手順については [micro-ROS tutorial](https://micro-ros.github.io/docs/tutorials/advanced/tracing/) と [lttng.org](https://lttng.org/docs/) をご覧ください。

#### Python モジュールのインストール
必要なモジュールをインストールします。

```
$ pip install pandas fire
```

### Fork 版 Foxy のビルド

Fork 版 Foxy のビルドにあたり、専用の ros2.repos を用意してあります。  
以下のコマンドを実行し、Fork 版 ROS2 のビルドをしてください。


```bash
$ mkdir -p ~/ros2_foxy_fork/src
$ cd ~/ros2_foxy_fork
$ wget https://gist.githubusercontent.com/hsgwa/bf2ca762072fa87a86df3e13a0d8b2d5/raw/bebcf6675c84f233ab4a50531161316769ad0d17/ros2.repos
$ vcs import src < ros2.repos
$ colcon build --symlink-install
```

`vcs import`は上書きは行わないコマンドです。  
バグ fix や機能拡張などがあった際には、以下のコマンドで各リポジトリを最新の状態に更新してください。
```bash
$ find src/ -name .git | xargs -I@ dirname @ | xargs -I@ -P8 sh -c 'cd @ && git pull'
```

### flamegraph.pl のインストール

flamegraph への出力を行う際には、flamegraph.pl をインストールする必要があります。

ここでは、`~/.local/bin` にインストールします。

```bash
$ wget https://raw.githubusercontent.com/brendangregg/FlameGraph/master/flamegraph.pl -O ~/.local/bin/flamegraph.pl && chmod +x $_
```

アンインストールする際は、以下のコマンドを実行してください。

```bash
$ rm ~/.local/bin/flamegraph.pl
```
