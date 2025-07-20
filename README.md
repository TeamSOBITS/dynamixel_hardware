<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# Dynamixel Hardware

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#環境構築">環境構築</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#設定方法">設定方法</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>



<!-- 概要 -->
## 概要

本リポジトリは[ROBOTIS Dynamixel](https://emanual.robotis.com/docs/en/dxl/)アクチュエタを動作させるための[`ros2_control`](https://github.com/ros-controls/ros2_control)の[`SystemInterface`](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp)を提供します.

`ros2_control`のアーキテクャャり全のDynamixelアクチュータ対応きると思れれます．

> [!IMPORTANT]
> オリジルルのパッケージと異り本フォークは異なるアクチュエータにおける複数制御方法に対応しています．
また，ギア比の設定やオフセットにも対応しています．
その一方，制御方法の切り替えはできなくなっております．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- セットアップ -->
## セットアップ

ここで，本レポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System  | Version |
| --- | --- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill |
| Python | 3.10 |

> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)に参照してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法

1. ROSの`src`フォルダに移動します．
    ```sh
    $ cd ~/colcon_ws/src/
    ```

2. 本レポジトリをcloneします．
    ```sh
    $ git clone https://github.com/TeamSOBITS/dynamixel_hardware
    ```

3. レポジトリの中へ移動します．
    ```sh
    $ cd dynamixel_hardware/
    ```

4. 依存パッケージをインストールします．
    ```sh
    $ bash install.sh
    ```

5. パッケージをコンパイルします．
    ```sh
    $ cd ~/colcon_ws/
    $ colcon build --symlink-install
    $ source ~/colcon_ws/install/setup.sh
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 設定方法 -->
## 設定方法

1. ロボットのURDFにアクチュエータのコントローラを設定します．

| パラメータ | 型 | 一例 | 説明 |
| --- | --- | --- | --- |
| port_name | string | /dev/ttyUSB0 | USBポート名 |
| baud_rate | int | 1000000 | Dynamixelの通信速度 |
| use_dummy | bool | true | ダミーアクチュエータを使用するかどうか |
| id | int | 1 | DynamixelのID |
| control_mode | int | 0 | 制御モード |
| interface | string | rs | インターフェース |

制御モードは以下の通りです．
| `control_mode` | 値 |
| --- | --- |
| Position Control | 0 |
| Velocity Control | 1 |
| Torque Control | 2 |
| Current Control | 3 |
| Extended Position Control | 4 |
| Multi Turn Control | 5 |
| Current Based Position Control | 6 |
| PWM Control | 7 |

インターフェースは以下の通りです．
| `interface` | 値 |
| --- | --- |
| RS485 | rs |
| TTL | ttl |

`Goal_Current`のようなDynamixel Workbenchから提供されているパラメータも設定できます．
詳しくは，[Dynamixel Workbench](https://github.com/ROBOTIS-GIT/dynamixel-workbench/blob/humble/dynamixel_workbench_toolbox/src/dynamixel_workbench_toolbox/dynamixel_item.cpp#L27-L119)にてご確認ください．

設定の一例はこちらとなります．
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<robot name="robot_name" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="controllers">

    <ros2_control name="dynamixel_control" type="system">
      <hardware>
        <plugin>dynamixel_hardware/DynamixelHardware</plugin>
        <param name="port_name">/dev/ttyUSB0</param>
        <param name="baud_rate">1000000</param>
        <!-- <param name="use_dummy">true</param> -->
      </hardware>

      <!-- 位置制御 -->
      <joint name="joint_1">
        <param name="id">1</param>
        <param name="control_mode">0</param>
        <!-- <param name="gear_ratio">2.0</param> -->
        <param name="interface">rs</param>
        <command_interface name="position">
          <param name="min">-3.141592654</param>
          <param name="max">3.141592654</param>
        </command_interface>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>

      <!-- 速度制御 -->
      <joint name="joint_2">
        <param name="id">2</param>
        <param name="control_mode">0</param>
        <param name="interface">rs</param>
        <command_interface name="velocity"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>

      <!-- 力により位置制御 -->
      <joint name="joint_3">
        <param name="id">3</param>
        <param name="control_mode">6</param>
        <param name="Goal_Current">50</param>
        <param name="interface">rs</param>
        <param name="gear_ratio">-59.504383354</param>
        <command_interface name="position"/>
        <state_interface name="position">
          <param name="initial_value">0.0</param>
        </state_interface>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      ...

    </ros2_control>
  </xacro:macro>
</robot>
```

2. 次は`controller manager`の設定となります．使用されるコントローラ・アクチュエータの数に応じて設定を変更してください．
```yaml
/**/controller_manager:
  ros__parameters:
    update_rate: 10  # Hz

    velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

/**/velocity_controller:
  ros__parameters:
    joints:
      - joint_2

/**/joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_3

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    allow_partial_joints_goal: true
```

3. 最後に，設定したパラメータ等をロボットを実行する際に，立ち上げます．
```py
robot_description = os.path.join(get_package_share_directory(
    'sobit_light_description'), 
    'robots.urdf.xacro'
)
robot_description_config = xacro.process_file(
    robot_description
)

controller_config = os.path.join(get_package_share_directory(
    'robot_control'),
    'config',
    'real_controllers.yaml'
)

controller_manager = Node(
    package="controller_manager",
    executable="ros2_control_node",
    namespace=robot_name,
    parameters=[controller_config],
    remappings=[
        ("controller_manager/robot_description", "robot_description"),
    ],
    output="screen",
)

joint_state_broadcaster = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller',
        '--set-state', 'active',
        '--controller-manager', robot_name+'/controller_manager',
        'joint_state_broadcaster'
    ],
    output='screen'
)

joint_trajectory_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller',
        '--set-state', 'active',
        '--controller-manager', robot_name+'/controller_manager',
        'joint_trajectory_controller'
    ],
    output='screen'
)

velocity_controller = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller',
        '--set-state', 'configured',
        '--controller-manager', robot_name+'/controller_manager',
        'velocity_controller'
    ],
    output='screen'
)

robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="robot_state_publisher",
    namespace=robot_name,
    parameters=[
        {"frame_prefix": robot_name + '/'},
        {"robot_description": robot_description_config.toxml()},
    ],
    output="screen",
)
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- マイルストーン -->
## マイルストーン

- [-] 異るアクチュエータの制御方法設定に対応
- [-] ギア比の設定
- [] 複数御御方法の切り替え

現時点のバッグや新規機能の依頼を確認するために[Issueページ][issues-url] をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 参考文献 -->
## 参考文献

* [Dynamixel Hardware](https://github.com/dynamixel-community/dynamixel_hardware)
* [Dynamixel Workbench](https://github.com/ROBOTIS-GIT/dynamixel-workbench/)
* [ROS Humble](https://docs.ros.org/en/humble/index.html)
* [ROS2 Control](https://control.ros.org/humble/index.html)
* [ROS2 Control Gazebo](https://github.com/ros-controls/gz_ros2_control)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/dynamixel_hardware.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/dynamixel_hardware/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/dynamixel_hardware.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/dynamixel_hardware/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/dynamixel_hardware.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/dynamixel_hardware/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/dynamixel_hardware.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/dynamixel_hardware/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/dynamixel_hardware.svg?style=for-the-badge
[license-url]: LICENSE
