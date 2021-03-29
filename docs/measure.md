## 測定方法

LTTng を使用して測定する際には、セッションを作成する必要があります。  
セッションを作成せずにアプリケーションを実行しても、トレース結果はファイルに保存されません。

ここでは、CUI からセッションの作成および測定をする方法と、
launch ファイルからセッションを作成および測定する方法 を説明します。  
どちらの方法も ros2 tracing を踏襲した使い方になります。

### CUI から利用する場合
ノードやコールバックの基本的な情報は、起動時のみに出力されるトレースデータを利用しています。  
そのため、測定する際の順序は以下の順で行う必要があります。

1. セッションの作成
2. アプリケーションの実行・測定

また、終了時は逆の順序で順次終了させることを推奨します。

セッションの作成は以下のコマンドを実行します。

```
$ ros2 trace -k -s your_session_name
UST tracing enabled (17 events)
kernel tracing disabled
context (3 names)
writing tracing session to: /home/hasegawa/.ros/tracing/your_session_name
pres enter to start...
```

`Enter`キーを押すことでトレースが開始し、再びエンターを押すとトレースが終了します。

トレース開始後、測定対象のアプリケーションを別ターミナルで実行してください。

<br />

なお、rostime を使用する際には新規ターミナルでセッションと記録用のノードの起動が必要です。

```
$ ros2 trace -k -u rostime:ros_time -s your_session_name_rostime
```

```
$ ros2 run clock_recorder recorder
```


#### オプションの説明
ros2 trace は他にもいくつかのオプションが用意されています。  
`ros2 trace -h`で確認できますが、ここでは関連するオプションの説明を加えておきます。

| オプション             | 説明                                                           |
|------------------------|----------------------------------------------------------------|
| -s SESSION_NAME        | セッション名。                                                 |
| -p PATH                | トレースデータの保存先。デフォルト：~/.ros/tracing/            |
| -u [EVENT [EVENT ...]] | トレース対象のユーザーランドのイベント名。空設定により無効化。 |
| -k [EVENT [EVENT ...]] | トレース対象のカーネルランドのイベント名。空設定により無効化。 |
| -l                     | 有効なイベント名の一覧を表示。                                 |

本ツールは ROS2 レイヤー（ユーザーランド）のイベントのみを取り扱っているため、  
カーネルランドのイベントは軽量化のために全て無効化しています。

---

### launch から利用する場合
CUI から実行する際には、セッションの開始のために別のターミナルを開き、Enter を入力する手間がありました。  
launch ファイルに記述することで、単一のコマンドでこれらの手順を実行できます。

#### Launch ファイルの修正

ros2 tracing にはセッションの開始用の api が実装されています。  

セッションの開始を既存の launch ファイルに加える際には 以下の行を加えてください。

```
from tracetools_launch.action import Trace

def generate_launch_description():
    return launch.LaunchDescription([
        Trace(
            session_name='your_session_name', # ~/.ros/tracing/your_session_name にトレース結果が保存される
            events_kernel=[] # カーネル関連のトレースを無視する
        ),

        ## rostime を使用する際はコメントアウト
        # Trace( 
        #     session_name='your_session_name_rostime',
        #     events_kernel=[],
        #     events_ust=['rostime:ros_time']
        # ),
        # launch_ros.actions.Node(
        #      package='clock_recorder', executable='recorder', output='screen'
        #  ),

        launch_ros.actions.Node(...),
        ...]
```

#### 引数の説明

| 引数          | 説明                                                                                                     |
|---------------|----------------------------------------------------------------------------------------------------------|
| session_name  | LTTng のセッション名。                                                                                   |
| events_kernel | セッションに記録させるカーネルのイベント名。イベントを記録させない場合は[]を指定。                       |
| events_ust    | セッションに記録させるユーザーランド（ROS レイヤー）のイベント名。イベントを記録させない場合は[]を指定。 |
| base_path     | トレース結果の保存先。 デフォルトでは`~/.ros/trace/`にトレース結果が保存されます                                       |
|               |                                                                                                          |
