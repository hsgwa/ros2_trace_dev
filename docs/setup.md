## インストール手順

本ツールである Fork 版 Foxy は各環境でビルドする必要があります。

基本的なインストール方法は [Building ROS 2 on Linux ](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/)と同じになります。  
ここでは、 Ubuntu Focal Fossa (20.04) 64-bit を想定します。

* TOC
{:toc}

### システムのセットアップ

#### LTTng のインストール

ROS2 レイヤーのトレースポイントも利用するため、  
事前に LTTng のインストールが必要です。

```
apt-add-repository ppa:lttng/stable-2.12
apt-get update
apt-get install lttng-tools
apt-get install lttng-modules-dkms
apt-get install liblttng-ust-dev
apt-get install python3-lttngust
```

#### ビルド関連パッケージのインストール

[System setup](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/#system-setup) にならい、foxy をビルドに必要な環境をセットアップします。  
以下の章のコマンドを実行してください。

```
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev
```


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

`vcs import`は上書きは行わないません。
バグ fix や機能拡張などがあった際には、以下のコマンドで各リポジトリを最新の状態に更新してください。
```bash
$ vcs pull src
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
