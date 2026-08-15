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
`/<ロボット名>/joint_states` を状態として受け取ります。

このリポジトリでできること
- CoRE ステージの読み込み（SDF ワールド）
- オムニホイールロボットの生成（最大 8 台）
- フライングディスクの装填（1 台につき 20 枚）
- ブラウザ経由でのロボット操縦（キーボード・ゲームパッド対応、キーコンフィグ機能付き）
- 720p（1280x720）カメラ映像のリアルタイム配信（JPEG 圧縮、rosbridge 経由）
- 試合管理機能（カウントダウン、HP 表示、個人戦/チーム戦対応）

## 必要なもの
1. Docker
1. GPU（無くても動きますが、カメラのレンダリングがソフトウェア描画になり重くなります）

Isaac 版と違い、Isaac Sim のイメージも NGC のアカウントも要りません。
シミュレータは Docker イメージのビルド時に GitHub Releases から取得します
（既定は **v1.0.4**）。

## 使い方

1. このリポジトリをクローンする
   ```bash
   git clone https://github.com/TKG-Tou-Kai-Group/CoRE-jp-Unity-ROS2-packages.git
   ```

1. サブモジュールをセットアップする
   ```bash
   cd CoRE-jp-Unity-ROS2-packages
   git submodule update --init --recursive
   ```

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
     ロボットの生成が終わると、続けてフライングディスク 20 枚がシュータへ積まれます。

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
     映像・操作ともに rosbridge（ポート 9090）のみで通信します。

     補足1：html 内の `SERVER_ADDRESS` を所望の IP アドレスに変更することで、
     同一 LAN の別の PC から操作することができます。

     補足2：この html では、CompressedImage（JPEG 圧縮済み映像）をサブスクライブし、
     Joy メッセージをパブリッシュしています。適宜トピック名を変更することで
     別の操作画面やロボットの操作に活用できます。

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

## Isaac 版からの変更点

シミュレータが変わったことで、対応する仕組みが次のように置き換わっています。

| 役割 | Isaac 版 | Unity 版 |
|---|---|---|
| シミュレータ起動・ステージ読み込み | `isaac_ros2_scripts/launcher_with_reset`（USD） | シミュレータは別プロセス。`core_sim_utils/load_world` が SDF ワールドを読ませる |
| ROS 2 との接続 | Isaac Sim 内蔵 | ROS-TCP-Endpoint（`ros_tcp_endpoint`） |
| ロボットの生成 | `isaac_ros2_scripts/spawn_robot` | `simulation_ros2_utils/spawn_entity`（`spawn_entity` サービス） |
| ディスクの配置 | `isaac_ros2_scripts/add_usd`（20 枚入り USD） | `core_sim_utils/spawn_flying_discs`（1 枚の URDF を 20 個 spawn） |
| センサ定義 | `sample_robot_description/isaac/*.isaac.xacro`（`<isaac>`） | `sample_robot_description/simulation/*.simulation.xacro`（`<simulation>`） |
| 接触摩擦 | `<material>` 内の `<isaac_rigid_body>` | `<robot>` 直下の `<collision_material>` を `<collision>` から名前で参照 |
| 試合リセット | 共有メモリ `isaac_sim_reset` | `reset_simulation` サービス（`SCOPE_STATE`） |
| ステージの形式 | `meshes/USD/core_stage.usd` | `worlds/core_stage.world`（SDF）＋ `meshes/STL/core_stage_NN.stl` |

センサのトピック名も変わっています。Isaac はリンクの親子関係がトピック名に入り
`/<ロボット名>/base_link/camera_link/image_raw` でしたが、Unity 版は
`/<ロボット名>/<リンク名>/image_raw` の 1 階層です。装甲板の接触も
`/<ロボット名>/armor1_link/contact`（`std_msgs/Bool`）になります。
`tools/*.html` と launch の remap はこの新しい名前に合わせてあります。

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

映像は JPEG 圧縮（品質 50）で配信しており、720p 10FPS 2 台同時接続で約 1.6MB/s（約 13Mbps）の
帯域を使用します。

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
