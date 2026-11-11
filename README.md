# shin-spider-ros

ROS package for the Processing spider controller port (ROS パッケージ名: `shin_spider`。catkin はハイフン不可のため、リポジトリ名と異なる)。

## 対応 ROS

| ブランチ | ROS | OS | 状態 |
|---|---|---|---|
| `main` | Noetic (ROS1) | Ubuntu 20.04 | 現行 |

## 概要

Processing 版 `ExercisesInCreativeRobotics2-main/Processing/original_spider_controller` の
**描画以外の内部処理** を ROS1 Noetic (roscpp, C++17) に移植したパッケージ。

- クラス構成は Processing と同じ: `Robot`, `RobotLeg`, `RobotHand`, `Motion`, `Target`, `MotionList`
- `leg_motion_commands` / `hand_motion_commands` の全動作 (test 動作含む) を移植
- `gamepad_command` のロジックを `sensor_msgs/Joy` で駆動
- OpenCM (`OpenCM/OpenCMSpiderMotion_v2.0_XC`) へ `[v,..]` → `[k,..]` → `[m,..]` を 2 ms 間隔で送信
- 描画 (3D 表示, camera, SubWindow, GIF, screenshot) とキーボード操作は移植しない

## ビルド / 実行

```bash
cd ~/catkin_ws/src
git clone https://github.com/Harlequinade1123/shin-spider-ros.git
cd ..
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -y
# catkin_tools (catkin build) を使うワークスペースでは catkin_make は使えない
catkin build shin_spider
source devel/setup.bash
roslaunch shin_spider shin_spider.launch                    # /dev/ttyUSB0, 57600
roslaunch shin_spider shin_spider.launch dry_run:=true      # 実機なし
```

シリアルが開けない場合は Processing の "Serial failed" と同様に **dry-run** で動き続ける
(送信内容は ROS_INFO と `~commands` (std_msgs/String) に出る)。

## テスト

```bash
catkin build shin_spider --no-deps --make-args tests
./devel/lib/shin_spider/shin_spider_test_kinematics   # 互換関数 / IK-FK / 状態遷移 / Joy / ファーム形式
./devel/lib/shin_spider/shin_spider_test_golden       # Processing(Java) 出力との完全一致
./devel/lib/shin_spider/shin_spider_test_pty          # pty ダミーシリアル結合試験
# または: catkin run_tests shin_spider --no-deps
```

- golden: `test/golden/golden.txt` (`tools/java_golden/run.sh` が `reference/processing/` の元の .pde を
  Java スタブでコンパイルして生成)。ファイルが無い場合 golden テストは SKIP。
  再生成は `tools/java_golden/run.sh` (JDK が必要。Processing 4 同梱の JDK も探す)。
  別の .pde を使うなら `PDE_DIR=<dir> tools/java_golden/run.sh`。出力先は第 1 引数で指定できる。
- `shin_spider_dump [case ...]` で C++ 側を同じ形式で出力できる (diff 用)。
  case 名の例: `walkingMotion(0,3)`, `walkingMotion(steering=1.5707964,mode=0)`,
  `sweepingMotion(-20)`, `seq[walkingMotion(0.0,1);resetMotion(0.0)]`

## ノード `shin_spider_node`

| 種別 | 名前 | 内容 |
|---|---|---|
| sub | `/joy` (`~joy_topic`) | sensor_msgs/Joy |
| pub | `~commands` | 送信した各行 (std_msgs/String, 改行なし) |
| param | `~device_name` | `/dev/ttyUSB0` |
| param | `~baudrate` | `57600` (Processing 既定 = BT-210/Serial2)。USB(Serial) 直結なら `115200` |
| param | `~dry_run` | `false` |
| param | `~reconnect` / `~reconnect_interval` / `~reconnect_wait` | `true` / `1.0` s / `2.0` s。書き込み失敗・デバイス消失(切断)を検知して close し、一定間隔で開き直す。起動時に開けなかった場合も同様 (kingspider の ensureSerialConnected 相当。再接続時もモーション状態は維持) |
| param | `~loop_rate` | `100` (Processing `frameRate(100)`) |
| param | `~axes/*`, `~buttons/*`, `~hat/*` | Joy 割り当て (`config/shin_spider.yaml`) |

1 フレーム (10 ms) ごとに Processing の `draw()` と同じ順で
`readGamePadCommand()` → 動作中 Motion の `next()` → `[v]`,2ms,`[k]`,2ms,`[m]`,2ms → ロボットモデル更新。
`millis()` は `ros::Time` (ノード起動からの ms) に置換。

## Joy 割り当て (既定: Xbox パッド + Linux xpad + joy_node)

Processing (gamecontrolplus / DirectInput) の変数の意味:
`slider_LX`/`slider_RX` = スティック上下 (**下が +**)、`slider_LY`/`slider_RY` = 左右 (**右が +**)、
`slider_FR` = トリガ (**L が +, R が −**)、値は `int(value*100)`。
joy_node は「上 = +1」「左 = +1」なので scale を **−100** にして符号を合わせている。

| Processing 変数 | Joy | 変換 | 用途 (gamepad_command) |
|---|---|---|---|
| slider_LX | axes[1] (LS 上下) | int(a*−100) | 上: walking 前進(mode 0) / 下: 後退(mode 3) / 斜め: mode 1,2,4,5 |
| slider_LY | axes[0] (LS 左右) | int(a*−100) | 横歩き (sideWalking 0/1) |
| slider_RX | axes[4] (RS 上下) | int(a*−100) | 上: jumping / 下: backJumping |
| slider_RY | axes[3] (RS 左右) | int(a*−100) | jumping 左右 (1/2) |
| slider_FR | axes[2] LT, axes[5] RT | int((pressL−pressR)*100), press=(1−a)/2 | ±50 超で 90° 方向転換 (rotationalMotion90) |
| button_A/B/X/Y | buttons 0/1/2/3 | 押下 +1 / 離す −1 | 手を振る / 薙ぎ払い(LB:−20, RB:+20) / 収納 / リセット |
| button_LB/RB | buttons 4/5 | 同上 | 両押しで `[n]` (トルク ON) |
| button_Back/Start/LA/RA | buttons 6/7/9/10 | 同上 | 未使用 (元コードも未使用) |
| hat_XY | axes[6] (左 +), axes[7] (上 +) | POV コード 0..8 | 上(2): spread / 下(6): shrink / 左(8): 回転 −45° / 右(4): 回転 +45° |

- joy_node はトリガ軸を一度触るまで 0.0 を返すため、未操作のトリガは「離している(+1)」とみなす。
- hat コード: 1=左上, 2=上, 3=右上, 4=右, 5=右下, 6=下, 7=左下, 8=左 (gamecontrolplus と同じ)。
- 別のパッドは `config/shin_spider.yaml` を書き換える (FR を単一軸にする場合は `mode: axis`)。

## ファームウェア形式の検証 (OpenCMSpiderMotion_v2.0_XC.ino)

`src/FirmwareProtocol.cpp` に `readCommandFromSerial()` の移植 (`parseLikeFirmware`) と検査
(`validateLine`) を実装し、ノードは送信前に全行を検査 (異常は ROS_WARN、送信自体は Processing 同様に行う)。

| 項目 | ファーム | 本パッケージ | 結果 |
|---|---|---|---|
| 行形式 | `[` で始まり `]` で終わる, `readStringUntil('\n')` | `[v,id,val,...]\n` (CR なし) | 一致 |
| コマンド文字 | i s m e p n f v k x z d b | v, k, m, n のみ送信 | 一致 |
| 要素数上限 | `elements[2*18+1]` = 37 | 最大 25 (4 脚 × 3 関節 × 2 + 1) | OK |
| ID | 1..18 | 脚 1..12, 腕 13..18 | 一致 |
| 位置 | 0..4095 (範囲外の要素は無視) | Target が 0..4095 外を出力しない | 一致 |
| 速度 / 加速度 | 0..32767 | Target が 0..32767 外を拒否, 生成値の最大 2800 | 一致 |
| ボーレート | Serial(USB)=115200, Serial2(BT-210)=57600 | 既定 57600 | **注意**: USB 直結時は 115200 |

### 不一致・注意点

1. **位置リミットレジスタ**: ファームは `setup()` で ID ごとに Max/Min Position Limit
   (脚第2,3関節 750..3350, 腕 13/16: 1050..3050, 14/17: 750..3350) を書き込む。
   生成値のうち walking / sideWalking / jumping 等で第3関節 (ID 3,6,9,12) に **651, 690, 729, 3404, 3443** が出る (750..3350 外)
   (`shin_spider_test_kinematics` の `Firmware.AllGeneratedLinesValid` が一覧を出力, 計 108 件)。
   リミットが有効ならサーボ側でクランプ/拒否される可能性がある。
2. ステアリング関節 (ID 1,4,7,10) は Max=Min=0 が書かれる。ただしファームは
   **トルク ON 後に** EEPROM 領域のリミットを書いているため、X シリーズでは書き込み自体が失敗している
   可能性が高い (未検証)。検査ではこの 0/0 を「リミット無し」として扱う。
3. ファームの `,` 数え上げループは `,` の直後 1 文字を読み飛ばすため空要素 (`,,`) を正しく数えない。
   本パッケージは空要素を送らないので影響なし。
4. `[v]`/`[k]` を受けるたびにファームは USB 側へ `"vel"`/`"acc"` を print する (動作には無関係)。

## Processing との対応・意図的な差分

- float 演算は Java と同じく各演算を float で丸める (`-ffp-contract=off`)。`sin/cos/atan2/sqrt` は
  double で計算して float に丸める (Processing と同じ)。`(int)` キャストは Java 準拠 (NaN→0)。
- `str(float)` は Java `Float.toString` 互換 (動作名 `sweeping_moiton_-20.0` など。typo も原文のまま)。
- 元コードの不具合も挙動として保持: `calcAndGetIK2DOF` の制限チェックが `ans_rad_thetas` でなく
  メンバ `rad_thetas` を見る / `calcAndGetThetasFromRatio` のクランプループが 0 回 /
  `motionStart()` は新規作成した動作を ID 競合に関係なく開始する。
- `calcAndGetIK2DOF` がパラメータ未設定時に null を返す代わりに {0,0,0} を返す (通常起こらない)。
- `setTargetVelocityAndAcceleration` を target 無しで呼ぶと Processing は例外、C++ はエラー表示のみ。
- `testLegMotion(2|3)` は元コードでも範囲外アクセス (leg+2)。C++ では `std::out_of_range`。
- キーボード操作 (矢印キー, '0', 's', is_simulation 時の文字キー) は移植しない。

## ファイル

- `include/shin_spider/`, `src/`: ROS 非依存ライブラリ `shin_spider_core`
  (時計 `Clock::millis()` と出力 `CommandSink` を注入) + `shin_spider_node.cpp`
- `src/MotionTestHook.cpp`: テスト用フック (`test_hook::runCase`, `stepMotionAllIndices`)
- `tools/shin_spider_dump.cpp`: golden 形式ダンプ, `tools/java_golden/`: Java 版 golden 生成
- `reference/processing/`: 移植元の Processing 原典 (golden 生成に必要な 9 ファイルのみ、改変なし)
- `test/`: gtest 3 本, `test/golden/golden.txt`

## License

MIT
