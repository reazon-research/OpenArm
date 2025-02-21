# OpenArm Bilateral Control

このレポジトリはレアゾンHI研究所で開発しているOpenArmでバイラテラル制御を行うものです。

## クイックスタート

最初にこのプロジェクトのルートディレクトリに入ります。

```console
$ cd teleoperation/
```
`rosdep` で必要な依存関係をインストールします。

```console
$ # 初めてrosdepを使う場合は初期化します（要初回のみ）
$ sudo rosdep init

$ # 依存関係をインストールします
$ sudo apt-get update
$ rosdep update
$ rosdep install -i --from-path src/reazon_bilateral_aloha/ -y
```
ビルドする

```console
$ colcon build --packages-select openarm_bilateral && source install/setup.bash
```

SocketCANを有効化(Canable有効化)
リーダーのip名をcan0 フォロワーのip名をcan1としてください。


```console
$ sudo pkill -f slcand 
#リーダ側のusb2canをpcに接続
$ sudo slcand -o -c -s8 /dev/ttyACM0 can0
$ sudo ip link set can0 up type can bitrate 1000000
#フォロワ側のusb2canをpcに接続
$ sudo slcand -o -c -s8 /dev/ttyACM1 can1
$ sudo ip link set can1 up type can bitrate 1000000
```

バイラテラル制御を起動。
このとき初期位置まで移動してから制御を開始します。

```
$  ros2 launch openarm_bilateral bilateral_openarm_main.launch.py
```

## バイラテラル制御，ユニラテラル制御切り替え

src/bilateral_main.cpp内のFollowerNode，LeaderNodeクラス内のtimer_callback()内のDoControl()のコメントを外して，DoControl_u()を適用すればユニラテラル制御になります。

### `src/bilateral_main.cpp`

```cpp
void timer_callback()
{
    control_f_->DoControl();
    // control_f_->DoControl_u();
}
```

```cpp
void timer_callback()
{
    control_l_->DoControl();
    // control_l_->DoControl_u();
}
```


## Caution

最初はリーダーとフォロワーのロボットの先端が真下を向くように設置してください。ゼロイチ初期化を行っています。ココからずれると重力補償などの誤差でロボットが予期しない動作をする可能性があります。


