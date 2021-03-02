## CUI ツール一覧

本ツールはアーキテクチャファイルの作成などで使用できる CUI ツールをいくつか提供しています。  
ここでは、それぞれの CUI ツールの一覧と使い方を説明します。


| コマンド名            | 概要                                                       |
|-----------------------|------------------------------------------------------------|
| trace_create_template | トレース結果からアーキテクチャファイルの生成               |
| trace_draw_node_graph | アーキテクチャファイルからノード図の作成                   |
| trace_list_target     | テスト対象のパス名の一覧を出力                             |
| trace_analysis        | テスト要件 yaml を元に、各グラフの作成とサマリ yaml の出力 |
| trace_collapse        | 解析結果から flamegraph.pl 向けのテキストを出力            |

上記のコマンドを利用するためには、以下のコマンドを実行してパスを通す必要があります。

```bash
$ source ~/ros2_foxy_fork/install/setup.bash
```

* TOC
{:toc}

<br />

### trace_create_template

トレース結果からアーキテクチャファイルの雛形を生成

アーキテクチャファイルの詳細については [ architecture ファイルの作成](./architecture.md) をご覧ください。

#### インターフェース

```bash
$ trace_create_template [/path/to/trace_result] [/path/to/architecture]
```

| 引数                    | 説明                                 |
|-------------------------|--------------------------------------|
| [/path/to/trace_result] | トレースした CTF ファイルへのパス    |
| [/path/to/architecture] | 書き出すアーキテクチャファイルのパス |


#### 実行例
```bash
$ trace_create_template ~/.ros/tracing/e2e_demo/ architecture.json
found converted file: /home/hasegawa/.ros/tracing/e2e_demo/converted
 [100%] [Ros2Handler]
```
```bash
$ ls ./architecture.json
architecture.json
```

<br />


### trace_draw_node_graph

アーキテクチャファイルからノード図の作成  
アーキテクチャファイルのチェックも行う。

#### インターフェース

```bash
$ trace_draw_node_graph [/path/to/architecture] [/path/to/png] ([target_path])
```

| 引数                    | 説明                                                                                       |
|-------------------------|--------------------------------------------------------------------------------------------|
| [/path/to/architecture] | 可視化対象のアーキテクチャファイルのパス                                                   |
| [/path/to/png]          | 書き出す png 画像のパス                                                                    |
| ([target_path])         | 強調するパスの名前。trace_list_target コマンドで出力されるパス名を指定可。オプション。 |


#### 実行例

##### アーキテクチャファイルの雛形を入力したケース
```br
$ trace_draw_node_graph architecture.json.template node_graph.png
0 end-to-end paths found.
5 nodes, 1 node paths found.
6 communication found.
9 callbacks found.
Failed to find start node. Please set [/target_path/start_node_name].
Failed to find end node. Please set [/target_path/end_node_name].
5 communications have no callback name. Please set [/nodes/publish/topic].
```
見つかった End-to-End のパス数、ノード数、通信数、コール瀑数を表示します。  
End-to-End のパスの発見にあたり、開始ノード名の終了ノード名が設定されていないこと。  
また、トピックをパブリッシュするコールバック名が指定されていないことをエラーとして出力しています。

[![ノード図](../imgs/node_graph_temp.png)](../imgs/node_graph_temp.png)
コールバック名が指定されず、パブリッシュするコールバックが不明な通信は点線で示されます。


##### target_path を指定しない場合
```br
$ trace_draw_node_graph architecture.json node_graph.png
3 end-to-end paths found.
5 nodes, 10 node paths found.
6 communication found.
9 callbacks found.
```
[![ノード図](../imgs/node_graph.png)](../imgs/node_graph.png)
正常にコールバック名が指定された通信は青の実線になります。  
開始ノードは水色、終了ノードはオレンジに塗りつぶしされます。



##### target_path を指定した場合
```br
$ trace_draw_node_graph architecture.json node_graph.png end_to_end_1
3 end-to-end paths found.
5 nodes, 10 node paths found.
6 communication found.
9 callbacks found.
```
[![ノード図](../imgs/node_graph_target.png)](../imgs/node_graph_target.png)
`target_path`に含まれるパスが赤く強調されます。

<br />

### trace_list_target

テスト対象のパス名の一覧を出力

#### インターフェース

```bash
$ trace_list_target [/path/to/architecture.json]
```

| 引数                         | 説明                                   |
|------------------------------|----------------------------------------|
| [/path/to/architecture.json] | 読み込むアーキテクチャファイルへのパス |

#### 実行例

```bash
$ trace_list_target architecture.json
found converted file: /home/hasegawa/.ros/tracing/e2e_demo/converted
 [100%] [Ros2Handler]
/topic1_0
/topic1_dds_0
TimerDependencyNode::TimerDependencyNode()::{lambda()#2}
TimerDependencyNode::TimerDependencyNode()::{lambda(std::unique_ptr<sensor_msgs::msg::Image>)#1}
TimerDependencyNode::TimerDependencyNode()::{lambda(std::unique_ptr<sensor_msgs::msg::Image>)#1}--TimerDependencyNode::TimerDependencyNode()::{lambda()#2}
end_to_end_0
timer_dependency_node_1
...
```

出力されるパスは以下のフォーマットになっています。

| パス名のフォーマット                    | 説明                                                                 |
|-----------------------------------------|----------------------------------------------------------------------|
| `/[topic_name]_0`                       | publish()実行直後から subscribe コールバック実行直前までのレイテンシ |
| `/[topic_name]_dds_0`                   | DDS レイヤーのみのレイテンシ                                         |
| `/[callback_symbol]`                    | コールバックの実行時間                                               |
| `/[callback_symbol]--[callback_symbol]` | コールバック間のレイテンシ                                           |
| `[node_name]_0`                         | ノードレイテンシ                                                     |
| `end_to_end_0`                          | End-to-End レイテンシ                                                |

<br />

### trace_analysis

テスト要件 yaml を元に、各グラフの作成とサマリ yaml の出力

#### インターフェース

```bash
$ trace_analysis [/path/to/input.yml] [/path/to/export_dir] [/path/to/trace_result] [/path/to/architecture.json]
```

| 引数                         | 説明                                   |
|------------------------------|----------------------------------------|
| [/path/to/input.yml]         | 読み込むテスト要件 yaml へのパス       |
| [/path/to/export_dir]        | 結果を出力するディレクトリへのパス     |
| [/path/to/trace_result]      | 解析対象のトレース結果へのパス         |
| [/path/to/architecture.json] | 読み込むアーキテクチャファイルへのパス |

#### 実行例
```bash
$ trace_analysis ./input.yaml . ~/.ros/tracing/e2e_demo/ ./architecture.json
found converted file: /home/hasegawa/.ros/tracing/e2e_demo/converted
 [100%] [Ros2Handler]
```

```
$ tree .
├── graph
│   ├── ActuatorDummy::ActuatorDummy()::{lambda(std::unique_ptr<sensor_msgs::msg::Image>)#1}-hist.png
│   ├── ActuatorDummy::ActuatorDummy()::{lambda(std::unique_ptr<sensor_msgs::msg::Image>)#1}-timeseries.png
│   ├── ...
│   ├── topic6_dds_0-hist.png
│   └── topic6_dds_0-timeseries.png
├── input.yaml
└── output.yaml
```

`[export_dir]/output.yaml` にテストサマリを出力し、  
`[export_dir]/graph/*`に可視化した全てのグラフを出力します。  
output.yaml のフォーマットについては[グラフの種類とグラフの見方](./how_to_read_graph.md) をご覧ください。

<br />


### trace_collapse

解析結果から flamegraph.pl 向けのテキストを出力

#### インターフェース

```bash
$ trace_collapse [/path/to/export_dir] [/path/to/trace_result] [/path/to/architecture.json]
```

| 引数                         | 説明                                                       |
|------------------------------|------------------------------------------------------------|
| [/path/to/export_dir]        | flamegraph.pl 向けのテキストを出力するディレクトリへのパス |
| [/path/to/trace_result]      | 解析対象のトレース結果へのパス                             |
| [/path/to/architecture.json] | 読み込むアーキテクチャファイルへのパス                     |

#### 実行例

```bash
$ trace_collapse . ~/.ros/tracing/e2e_demo/ ./architecture.json
found converted file: /home/hasegawa/.ros/tracing/e2e_demo/converted
 [100%] [Ros2Handler]
```

```bash
$ tree .
.
└── graph
     ├── end_to_end_0_collapsed.log
     ├── end_to_end_1_collapsed.log
     └── end_to_end_2_collapsed.log
```

flamegraph を生成する際は以下のコマンドを実行します。  
flamegraph.pl のインストールについては [インストール手順](./setup.md) をご覧ください。
```bash
$ flamegraph.pl ./graph/end_to_end_0_collapsed.log --countname ms > ./graph/end_to_end_0_collapsed.svg
```
```bash
$ tree .
.
└── graph
    ├── end_to_end_0_collapsed.log
    ├── end_to_end_0_collapsed.svg
    ├── end_to_end_1_collapsed.log
    └── end_to_end_2_collapsed.log
```
