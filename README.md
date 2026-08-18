# CoRE-jp-Unity-ROS2-packages

- サンプルロボット操縦デモ
  ![ros2_control_demo](figs/core_sim_multi_robot_test.gif)

このリポジトリは、[Unity_ROS2_Robot_Simulator](https://github.com/REACT-ROBOT/Unity_ROS2_Robot_Simulator) 上で
CoRE の競技環境を動かすための ROS 2 パッケージをまとめたものです。
[CoRE-jp-Isaac-Sim-ROS2-packages](https://github.com/TKG-Tou-Kai-Group/CoRE-jp-Isaac-Sim-ROS2-packages) の Unity 版にあたります。

シミュレータとのやり取りは [simulation_interfaces](https://github.com/ros-simulation/simulation_interfaces) の
サービス (`load_world` / `spawn_entities` / `set_simulation_state` / `reset_simulation` …) と
`topic_based_ros2_control` が担います。`topic_based_ros2_control` は hardware_interface を提供し、
ros2_control からのコマンドを `/<ロボット名>/joint_command` としてシミュレータへ送り、
`/<ロボット名>/joint_states` を状態として受け取ります。ros2_control が担うのは
**射出機構だけ**で、走行はシミュレータが `/<ロボット名>/cmd_vel` を直接購読します
(後述の「走行の駆動方式」を参照)。

このリポジトリでできること
- CoRE ステージの読み込み（SDF ワールド）
- ロボットの生成（最大 8 台）
- フライングディスクの都度生成（装填口に常に 1 枚、場に最大 24 枚）
- ブラウザ経由でのロボット操縦（キーボード・ゲームパッド対応、キーコンフィグ機能付き）
- 720p（1280x720）カメラ映像のリアルタイム配信（H.264、WebSocket 経由）
- 試合管理機能（カウントダウン、HP 表示、個人戦/チーム戦対応）

## 必要なもの
1. Docker
1. GPU（無くても動きますが、カメラのレンダリングがソフトウェア描画になり重くなります）

Isaac 版と違い、Isaac Sim のイメージも NGC のアカウントも要りません。
シミュレータは Docker イメージのビルド時に GitHub Releases から取得します
（既定は **v1.1.0**）。

> **注意**: 走行の駆動に **v1.1.0 以降が必要**です（`body_twist_drive` に対応した
> バージョン）。v1.0.5 以前では `cmd_vel` が無視され、ロボットが動きません。

## 使い方

1. このリポジトリをサブモジュールごとクローンする
   ```bash
   git clone --recursive https://github.com/TKG-Tou-Kai-Group/CoRE-jp-Unity-ROS2-packages.git
   cd CoRE-jp-Unity-ROS2-packages
   ```

   `--recursive` を付け忘れた場合や、あとから取得する場合は次を実行してください。
   ```bash
   git submodule update --init --recursive
   ```

   `colcon_ws/src` の以下 5 つはサブモジュールです。取得しないと `colcon build` が
   メッセージ型やコントローラを見つけられずに失敗します。

   | サブモジュール | 役割 |
   | --- | --- |
   | `simulation_interfaces` | シミュレータ操作の標準インタフェース (spawn / reset / state) |
   | `simulation_ros2_utils` | 上記を叩く CLI と、シナリオ試験用のクライアント |
   | `ROS-TCP-Endpoint` | シミュレータとの接続口 |
   | `topic_based_ros2_control` | ros2_control をトピック経由でシミュレータへ繋ぐ |

1. Docker イメージをビルドする
   ```bash
   cd docker
   ./build_docker_image.sh          # ROS 2 Jazzy  / Ubuntu 24.04
   ./build_docker_image.sh humble   # ROS 2 Humble / Ubuntu 22.04
   ```

1. Docker コンテナを立ち上げる
   ```bash
   ./launch_docker.sh
   ```

1. ROS 2 のソースコードをビルドする
   ```bash
   colcon build && source install/setup.bash
   ```

   > **注意**: `colcon_ws` はディストロ間で共有できません。humble と jazzy を行き来する
   > ときはビルド生成物を消してください（Python のバージョンが 3.10 / 3.12 で違うため、
   > 残っていると `UnsupportedTypeSupport` でメッセージ型のロードに失敗します）。
   > ```bash
   > rm -rf build install log && colcon build
   > ```

1. パッケージを起動する

   - シミュレータおよびステージの立ち上げ

     立ち上げたコンテナ内で以下のコマンドを実行してください。
     シミュレータ本体・ROS-TCP-Endpoint・ステージの読み込み・試合管理・rosbridge が
     まとめて立ち上がります。
     ```bash
     ros2 launch sample_robot_sim bring_up_core_stage.launch.py
     ```

     シミュレータを自分で起動したい場合や、別 PC で動かしている場合は
     引数で切り離せます。
     ```bash
     ros2 launch sample_robot_sim bring_up_core_stage.launch.py launch_simulator:=false
     ```

   - ロボットの生成

     別のターミナルから下記のコマンドを実行してください。
     ```bash
     docker exec -it core-unity-sim-jazzy /bin/bash
     ros2 launch sample_robot_sim sample_robot_1_spawn_for_core.launch.py
     ```
     `sample_robot_1_spawn_for_core.launch.py` の数字部分を変更すると別のロボットを
     生成できます。番号は 1 から 8 まで用意しています。
     フライングディスクは起動時には積まれません。`flying_disc_feeder` が装填口へ
     常に 1 枚だけ置き、撃つたびに次の 1 枚を作ります。

   - シミュレーションの開始／停止

     Isaac のシミュレータ左の三角ボタン／四角ボタンに相当します。
     GUI のボタンでも、下記のスクリプトでも操作できます。
     ```bash
     ./scripts/start_sim.sh   # 開始
     ./scripts/stop_sim.sh    # 停止
     ```

   - 操作インターフェースの立ち上げ

     `tools/robot_1_control.html` をブラウザで開いてください。
     `robot_1` から `robot_8` まであり、各ロボットに対応しています。

     操作は rosbridge（ポート **9090**）、映像は `operator_stream` の
     WebSocket（ポート **9091**）で受け取ります。映像は H.264 で、
     1 系統あたり約 0.7 Mbps です（rosbridge 経由の JPEG は 2.8 Mbps）。
     8 台ぶん同時に見ても 5.8 Mbps に収まります。

     `operator_stream` は `bring_up_core_stage.launch.py` が起動します。
     視聴者が居ない系統は符号化しないので、誰も見ていない機体のぶんは
     CPU を使いません。

     補足1：html 内の `SERVER_ADDRESS` を所望の IP アドレスに変更することで、
     同一 LAN の別の PC から操作することができます。映像用の 9091 も
     併せて共有してください。

     補足2：この html は映像を `operator_stream` の WebSocket から H.264 で受け取り、
     操作は rosbridge へ Joy メッセージとして送っています。`ROBOT_NAME` と
     `VIDEO_STREAM` を変えれば別のロボットに向けられます。

   - 操作方法

     画面右上の「Settings」ボタンからキーコンフィグの変更が可能です。
     設定はブラウザの LocalStorage に保存されます。

     キーボード（デフォルト）：

     | 操作 | キー |
     |---|---|
     | 前後左右移動 | 矢印キー |
     | 旋回 | A / D |
     | 射撃 | Space |
     | ターボ | Shift |
     | 俯瞰画像切替 | V |
     | リセット | R |
     | 速度調整 | PageUp / PageDown |

     ゲームパッド（PS コントローラ）：

     | 操作 | ボタン |
     |---|---|
     | 前後左右移動 | 左スティック |
     | 旋回 | 右スティック左右 |
     | 射撃 | RB (R1) |
     | ターボ | Back (Select) |
     | 俯瞰画像切替 | LB (L1) |
     | リセット | Start |
     | 速度調整 | 十字キー上下 |

## 走行の駆動方式

走行は**シミュレータが車体へ直接力を加えて**行います。`/<ロボット名>/cmd_vel`
(`geometry_msgs/Twist`、車体基準) をシミュレータが直接購読し、各物理ステップで
「指令速度 − 現在速度」を詰めるのに要る力とトルクを車体へ加えます。ros2_control も
`omni_wheel_controller` も経由しません。

以前はオムニホイール 4 輪で床の摩擦により走らせていましたが、1 輪あたり
フリーローラ 8 個 + ハウジング 2 個、4 輪で 40 リンクあり、これが PhysX ソルバの
コストの大半を占めていました。ロボット 2 台 + ディスク 40 枚で物理が実時間に
収まらなくなり、Unity が 1 フレームに 0.333 秒ぶん (`Time.maximumDeltaTime` の既定値)
の物理をまとめて回すため、**カメラ映像が 2.8 FPS まで落ちていました**。

置き換えによる実測値 (physics_hz 200、ロボット 2 台 + ディスク 40 枚):

| | オムニホイール | 直接加力 |
|---|---|---|
| カメラ映像 | 2.82 FPS | **10.00 FPS** |
| RTF | 0.94 | **1.000** |
| 8 台での FPS / RTF | 2.02 FPS / 0.66 | **9.97 FPS / 0.998** |
| 前後の指令追従 | 54% | **100%** |
| 左右の指令追従 | 54% | **100%** |
| 旋回の指令追従 | 82% | **97〜100%** |
| ロボット 1 台のリンク数 | 69 | **29** |

速度を代入するのではなく力で追従させているので、**他機に押されるし押し返せます**
(指令ゼロで静止している機体を 1.0 m/s で押して 2.82 m 動かせることを確認済み)。

速度・加速度の上限は
[simulation/sample_robot.simulation.xacro](colcon_ws/src/sample_robot_description/simulation/sample_robot.simulation.xacro)
の `<sensor type="body_twist_drive">` にあります。値は以前の `omni_wheel_controller` の
設定をそのまま引き継いでいます (1.0 m/s / 0.4 m/s²、1.5 rad/s / 0.8 rad/s²)。

> **注意**: 以前は車輪が滑って指令の 54% しか出ていませんでした。同じ指令値でも
> 実際の速度は約 1.85 倍になるので、競技の間合いが変わります。以前の実効速度に
> 合わせたい場合は `max_linear_velocity` を 0.54 前後まで下げてください。

車体を床から浮かせるために、車輪と同じ位置・同じ半径 (0.05 m) の摩擦 0 の球を
`base_frame_link` の `<collision>` として 4 個付けています
([urdf/base/base.urdf.xacro](colcon_ws/src/sample_robot_description/urdf/base/base.urdf.xacro)
の `base_skid`)。リンクは増えません。これが無いと車体が 5 cm 下がって床に直付きになり、
フィールドの構造物に引っ掛かって走れなくなります (実測: 上限の 156 N を掛けても速度 0)。
車輪の見た目もここでハウジングの STL を置いて再現しています (回転はしません)。

この駆動方式には **Unity_ROS2_Robot_Simulator 側の `body_twist_drive` 対応が必要**です。
対応していないバージョンでは `cmd_vel` が無視され、ロボットは動きません。

## Isaac 版からの変更点

シミュレータが変わったことで、対応する仕組みが次のように置き換わっています。

| 役割 | Isaac 版 | Unity 版 |
|---|---|---|
| シミュレータ起動・ステージ読み込み | `isaac_ros2_scripts/launcher_with_reset`（USD） | シミュレータは別プロセス。`core_sim_utils/load_world` が SDF ワールドを読ませる |
| ROS 2 との接続 | Isaac Sim 内蔵 | ROS-TCP-Endpoint（`ros_tcp_endpoint`） |
| ロボットの生成 | `isaac_ros2_scripts/spawn_robot` | `simulation_ros2_utils/spawn_entity`（`spawn_entity` サービス） |
| ディスクの配置 | `isaac_ros2_scripts/add_usd`（20 枚入り USD） | `core_sim_utils/flying_disc_feeder`（撃つたびに 1 枚生成、場に最大 24 枚） |
| センサ定義 | `sample_robot_description/isaac/*.isaac.xacro`（`<isaac>`） | `sample_robot_description/simulation/*.simulation.xacro`（`<simulation>`） |
| 接触摩擦 | `<material>` 内の `<isaac_rigid_body>` | `<robot>` 直下の `<collision_material>` を `<collision>` から名前で参照 |
| 試合リセット | 共有メモリ `isaac_sim_reset` | `reset_simulation` サービス（`SCOPE_STATE`） |
| ステージの形式 | `meshes/USD/core_stage.usd` | `worlds/core_stage.world`（SDF）＋ `meshes/STL/core_stage_NN.stl` |

センサのトピック名も変わっています。Isaac はリンクの親子関係がトピック名に入り
`/<ロボット名>/base_link/camera_link/image_raw` でしたが、Unity 版は
`/<ロボット名>/<リンク名>/...` の 1 階層です。
`tools/*.html` と launch の remap はこの新しい名前に合わせてあります。

| 中身 | トピック | 型 |
|---|---|---|
| 一人称カメラ | `/<ロボット名>/camera_link/image_raw/compressed` | `sensor_msgs/CompressedImage` |
| 俯瞰カメラ | `/<ロボット名>/top_view_camera_link/image_raw/compressed` | `sensor_msgs/CompressedImage` |
| 装甲板の被弾 | `/<ロボット名>/armorN_link/contact_count` | `std_msgs/Int32` |

被弾は真偽値ではなく**接触が始まった回数の累積**です。購読側は 2 回の受信の
差を取ります。立ち上がりを数える方式だと、途中でメッセージが 1 通落ちただけで
その被弾が永久に失われるためです。減っていたらリセットか再スポーンの合図なので、
差が負のときは被弾として数えないでください。

### カメラの形式

シミュレータは既定で JPEG を出します。生画像が要る場合は環境変数で切り替えます。

```bash
CORE_CAMERA_FORMAT=raw ros2 launch sample_robot_sim sample_robot_1_spawn_for_core.launch.py
```

`raw` にすると `/<ロボット名>/camera_link/image_raw`（`sensor_msgs/Image`）に
変わります。`ros2 launch --show-args` には出ません。URDF を組み立てる xacro が
launch の実行前に走るため、launch 引数にできないからです。

**通常は既定のままで構いません。** JPEG のほうが速いためです。Unity から
ros_tcp_endpoint への経路は TCP 1 本で、生画像を 8 機ぶん（一人称と俯瞰で 16 本）
流すと溢れます。溢れ方は 1 フレーム内の publish 順、つまり機体の生成順に効くので、
番号の大きい機体ほど映像が落ちます。

8 台・960×540 での実測:

| 形式 | 全機の FPS | 送信キューの廃棄 |
|---|---|---|
| jpeg | 10.10（8 機とも） | 23 件 |
| raw | 8.80 〜 10.45 | 16,251 件 |

符号化はシミュレータ側のワーカースレッドで走るので、物理演算とは競合しません。
そのぶん映像は 1 周期（100 ms）遅れて出ますが、タイムスタンプは撮影時刻のままです。

### ステージの変換について

Unity のシミュレータは USD を読めないので、Isaac 版が `bring_up_core_stage.launch.py`
で開いている `meshes/USD/core_stage.usd` の中身を、
[tools/convert_core_stage_usd.py](tools/convert_core_stage_usd.py) で SDF + STL へ
変換したものを収録しています。中身は **2026 年フィールド**（`core1-2026-field.usd` の
payload）＋ 壁 4 枚で、位置関係は Isaac と同一です。

- フィールド 18.3 × 27.6 m、原点中心
- 壁は x = ±10 m / y = ±14 m、高さ 5 m
- 地面はシミュレータの組み込みシーンのものを使う

ロボットの開始位置は Isaac 版と同じ座標（x: ±4.5〜5.5、y: ±9.75〜11.25）です。

メッシュ形式に **STL** を選んでいるのは、シミュレータの SDF ローダが拡張子で読み方を
変えており、STL だけが URDF インポータと同じ「頂点単位で ROS→Unity 変換する」経路を
通るためです。Collada は GameObject の transform に回転を当てる実装で、その後の
SDF pose の代入（`ObjectSpawner.ApplyTransform`）に上書きされ、**90 度傾いて表示されます**。

STL は色を持たないので、マテリアルごとにファイルを分け、SDF 側の
`<material><diffuse>` で色を与えています（16 ファイル / 14 色）。

**視覚メッシュがそのまま当たり判定になります。** シミュレータは読み込んだメッシュの
全 `MeshFilter` に `MeshCollider` を付けるため、SDF ワールドに視覚用と衝突用を分ける
経路はありません。既定ではボルト類（外形 5 cm 未満、4996 メッシュ / 131 万三角形）を
落として 168 万三角形にしてあります。全部入りにしたい場合や、さらに軽くしたい場合は
`--min-size` を変えて再生成してください。

```bash
pip install --target /tmp/usdlib usd-core numpy
PYTHONPATH=/tmp/usdlib python3 tools/convert_core_stage_usd.py \
    --usd /path/to/CoRE-jp-Isaac-Sim-ROS2-packages/colcon_ws/src/core_stage_description/meshes/USD/core_stage.usd \
    --out colcon_ws/src/core_stage_description \
    --min-size 0.05
```

RViz 用の `urdf/core_stage.urdf.xacro` も同時に生成されるので、
`ros2 launch core_stage_description marker_plate_description.launch.py` でも
同じ形状が出ます。

## シミュレータを外部ネットワークの参加者と共有

本シミュレータは ROS 2 の通信を rosbridge_suite によってポート 9090 で Pub/Sub することで
ブラウザ経由でロボットの操作ができるというものです。そのため、ポート 9090 を外部ネットワークの
参加者と共有することにより、同じシミュレーション環境を共有することができます。
必要なポートは **9090 の 1 つだけ**です。

かんたんにポートを共有するツールとして、[Secure Share Net](https://gsht.io/) というツールがあります。
こちらのツールを利用することで、外部ネットワークの参加者と気軽にシミュレーションすることが可能です。
以下に、Secure Share Net を使用する手順を示します。

必要なポートは操作用の **9090** と映像用の **9091** の 2 つです。
映像は H.264 で 1 系統あたり約 0.7 Mbps、8 台ぶんで約 5.8 Mbps です。

1. 現在シミュレータを起動している同じ PC で Secure Share Net を起動します。
1. Secure Share Net の管理画面から、「ローカルポート番号 (例: 25565)」の入力欄に「9090」を
   入力してください。（rosbridge_suite のデフォルトポート番号です。必要に応じて変更してください。）
1. 設定項目「プロトコル」を「TCP/UDP 両方」に設定します。
1. 設定などを変更・確認後、「公開する」を押します。
1. 表示された「○○.ssnetwork.io:○○○○○」が他のユーザーが参加できる「サーバーアドレス」です。
   このアドレスを共有したい参加者に教えてください。参加者は、html の `SERVER_ADDRESS` と
   `SERVER_PORT` をそれぞれ教えてもらったサーバーアドレスとポート番号に変更して html を
   開いてください。

## 参考

- シミュレータ本体: [Unity_ROS2_Robot_Simulator](https://github.com/REACT-ROBOT/Unity_ROS2_Robot_Simulator)
- シミュレータのサービス仕様: [docs/Simulation-Interfaces-Services-ja.md](https://github.com/REACT-ROBOT/Unity_ROS2_Robot_Simulator/blob/main/docs/Simulation-Interfaces-Services-ja.md)
- URDF からの摩擦設定: [docs/URDF-Collision-Material-ja.md](https://github.com/REACT-ROBOT/Unity_ROS2_Robot_Simulator/blob/main/docs/URDF-Collision-Material-ja.md)
- 単体ロボットのサンプル: [Unity_ROS2_sample](https://github.com/REACT-ROBOT/Unity_ROS2_sample)
