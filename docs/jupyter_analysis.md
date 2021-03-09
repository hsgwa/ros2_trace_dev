## jupyter での解析

本ツールはトレース結果の解析プログラムを python モジュールとして実装しており、  
python スクリプト内で測定結果を取得できます。

可視化作業から進めることで、整形したグラフの作成や、ボトルネックの特定に適しています。

ここでは、jupyter 上で測定結果を取得する方法を説明します。


* TOC
{:toc}

### jupyer の起動

jupyter-lab を起動します。

```bash
$ cd ros2_ws/src/e2e_demo/analysis
$ # pip install jupyterlab # 事前にインストールが必要
$ jupyter-lab
# ブラウザで http://localhost:8888 にアクセス
```

### サンプル一覧

デモには、以下のサンプルが付属しています。

- e2e_latency.ipynb ：End-to-End レイテンシの可視化
- node_latency.ipynb ：ノードレイテンシの可視化
- comm_latency.ipynb：通信レイテンシの可視化
- callback_duration.ipynb：コールバック実行時間の可視化
- cui.ipynb：CUI ツールの実行

可視化対象が決まっている場合は上記サンプルをコピーして扱うことを推奨します。

### python モジュールの使い方

各取得可能なレイテンシは、以下のような階層構造になっています。

```
End-to-End
├── Communication   通信＆スケジューリング：publish() から callback() 実行直前まで
│   └── DDS         通信：dds_write から on_data_available() 実行直前まで
└── Node            subscribe や timer コールバックの開始から publish するコールバックの終了まで
    ├── Callback    コールバックの実行時間：callback() 開始直前から callback() 終了直後まで
    └── Inter-callback コールバック間の時間： callback() 終了直後から callback() 開始直前まで
```

以降の内容では、上の階層から下の階層まで、順に測定結果を取得する方法を説明します。  
Callback, Communication, DDS の解析のみであれば、アーキテクチャファイルは雛形がそのまま利用可能です。

#### Path オブジェクトの API
レイテンシは Path オブジェクトの API を使用することで取得が可能です。  
Path オブジェクトは以下を内部で保持しています。

- レイテンシの確率分布 : `path.hist(binsize_ns: int)`
- レイテンシの時系列 : `path.timeseries(use_simtime: bool)`
- 基本統計量：`path.get_stats()`
  - 通信レイテンシ、コールバック間のレイテンシについてはロストも算出されます。
- パスを構成する下階層の Path : `path.child`

以降では End-to-End レイテンシからコールバックの実行時間までの測定結果を取得する例を示します。

#### End-to-End レイテンシの取得

```python
from tracetools_analysis.ros_model import ApplicationFactory

# architecture ファイルを読み込み、アプリケーションを再構築
app = ApplicationFactory.create_from_json('/path/to/arcitecture')

# 最初と最後の 1000ms の測定結果を除外し、定常状態のみの結果を取得する
app.import_trace(trace_path, start_transition_ms=1000, end_transition_ms=1000)

# レイテンシオブジェクトの取得
e2e_path = app.paths[0] # End-to-End パスは app.paths で取得

# パスの確認
e2e_path.child_names    # パスの中身は　path.child_names で取得
# sensor_dummy_node_2--no_dependency_node_0--sub_dependency_node_1--timer_dependency_node_1--actuator_dummy_node_1

# レイテンシの確立分布の取得
latency_ms, hist = e2e_path.hist(binsize_ns=1e6).get_xy()

# レイテンシの時系列の取得
system_time_ns, latency_ns = path.timeseries.get_xy()
simtime_ns_, latency_ns = path.timeseries.get_xy(use_simtime=True)

# 基本統計量の取得
e2e_path.get_stats()
# {'max': 320, 'min': 64, 'median': 150, 'mean': 161.3245898838004}

# 下の階層のオブジェクトを取得
node_path = e2e_path.child[0]  # 偶数インデックスはノードレイテンシ
comm = e2e_path.child[1]       # 奇数インデックスは通信レイテンシ
```

End-to-End のパスは複数存在するケースもあるため、 app.paths は配列になっています。

#### 通信レイテンシの取得

```python
from tracetools_analysis.ros_model import ApplicationFactory

# architecture ファイルを読み込み、アプリケーションを再構築
app = ApplicationFactory.create_from_json('/path/to/arcitecture')

# 最初と最後の 1000ms の測定結果を除外し、定常状態のみの結果を取得する
app.import_trace(trace_path, start_transition_ms=1000, end_transition_ms=1000)

# レイテンシオブジェクトの取得
comm = app.comms[0] # 通信レイテンシは app.comms で取得可
dds = comm.child[0] # child で下階層のレイテンシを取得可。ここでは DDS のレイテンシ。

# トピック名は topic_name の取得
comm.topic_name
# /topic4

# トピックを publish しているノード名の取得
comm.node_pub.name
# sub_dependency_node
comm.node_sub.name
# timer_dependency_node

# レイテンシの確立分布の取得
latency_ms, hist = comm.hist(binsize_ns=1e6).get_xy()
latency_ms, hist = dds.hist(binsize_ns=1e6).get_xy()

# 時系列のレイテンシの取得
system_time_ns, latency_ns = comm.timeseries.get_xy()
system_time_ns, latency_ns = dds.timeseries.get_xy()

# 基本統計量の取得
comm.get_stats()
dds.get_stats()
# {'min': 0.10470399999999999,
#  'max': 0.314112,
#  'median': 0.17638399999999999,
#  'avg': 0.18390958730158732,
#  'send': 63,
#  'lost': 0}
```

#### ノードレイテンシの取得

```python
from tracetools_analysis.ros_model import ApplicationFactory

# architecture ファイルを読み込み、アプリケーションを再構築
app = ApplicationFactory.create_from_json('/path/to/arcitecture')

# 最初と最後の 1000ms の測定結果を除外し、定常状態のみの結果を取得する
app.import_trace(trace_path, start_transition_ms=1000, end_transition_ms=1000)

# ノードオブジェクトの取得
node = app.nodes[0]

# ノード名の取得
node.name
# sub_dependency_node

# レイテンシオブジェクトの取得
node_path = node.paths[0]

# レイテンシの確立分布の取得
latency_ms, hist = node_path.hist(binsize_ns=1e6).get_xy()

# レイテンシの時系列の取得
system_time_ns, latency_ns = node_path.timeseries.get_xy()

# 基本統計量の取得
node_path.get_stats()
# {'max': 10, 'min': 10, 'median': 10, 'mean': 11.0}

# 下の階層のオブジェクトを取得
path.child # コールバックの実行時間またはコールバック間のレイテンシ
```

ノードのパスは複数存在するケースもあるため、 node.paths は配列になっています。

#### コールバック実行時間・コールバック間のレイテンシの取得

```python
from tracetools_analysis.ros_model import ApplicationFactory

# architecture ファイルを読み込み、アプリケーションを再構築
app = ApplicationFactory.create_from_json('/path/to/arcitecture')

# 最初と最後の 1000ms の測定結果を除外し、定常状態のみの結果を取得する
app.import_trace(trace_path, start_transition_ms=1000, end_transition_ms=1000)

# レイテンシオブジェクトの取得
node = app.nodes[0]
node_path = node.paths[0]
callback = path.child[0]    # 偶数インデックスはコールバックの実行時間
sched = path.child[1]       # 奇数インデックスはコールバック間のレイテンシ

# callback = app.callbacks[0]  # その他の取得方法
# sched    = app.scheds[0]
# callback = app.nodes[0].callbacks[0]
# sched    = app.nodes[0].scheds[0]

# レイテンシの確立分布の取得
latency_ms, hist = callback.hist(binsize_ns=1e6).get_xy()
latency_ms, hist = sched.hist(binsize_ns=1e6).get_xy()

# 時系列のレイテンシの取得
system_time_ns, latency_ns = callback.timeseries.get_xy()
system_time_ns, latency_ns = sched.timeseries.get_xy()

# 基本統計量の取得
callback.get_stats()
sched.get_stats()
# {'min': 10.10986328125,
#  'max': 10.27587890625,
#  'median': 10.193603515625,
#  'mean': 10.19248167936467,
#  'send': 121,
#  'lost': 0}
```

