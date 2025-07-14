# Introduction to RL Boy Project

Feishu Link：
https://hcn64ij2s2xr.feishu.cn/wiki/EKkhwVXq4ip1C3ks36fc7MlInZg?from=from_copylink


**RL-Boy Reinforcement Kid** is a small-sized open-source robot project and a full-stack open-source scaled-down humanoid robot project. Centered around running end-to-end reinforcement learning algorithms, it has 22 degrees of freedom, including waist, hands, and head. Among small-sized robots, it fully possesses the ability to run end-to-end reinforcement learning algorithms and imitation learning algorithms. Meanwhile, due to having upper limbs, it can also run open-source software packages related to VLA operations. Reinforcement Kid has a 1-degree-of-freedom head and an OLED screen, so it can display expressions well, and combined with AI models, it can implement dialogue and interactive actions.

## Features of the Reinforcement Kid:
1. **Ultra-small size and high degree of freedom:** The enhanced small robot has 22 degrees of freedom, with a rotatable waist and head, but is only 55 cm tall.
2. **End-to-end network drive:** The reinforcement agent can implement the RL deep reinforcement learning algorithm, while the upper limb can implement operations by running the VLA model, thus realizing the development of the cerebellum-cerebrum fusion algorithm.
3. **Open source of whole-link power builder:** In addition to open-sourcing robot mechanical design and BOM diagram materials, the robot's upper and lower computer software, as well as the onnx model deployment framework, have been open-sourced. The model deployment framework is fully synchronized with isacc and sim2sim scripts.
4. **Provide remote control equipment:** Provide servo-based dual-arm remote control software that can meet the data acquisition requirements for controlling and supporting the upper limbs and waist of the robot.

## Basic Performance of the Reinforcement Kid:

| **Degree of Freedom** | **Control System**       | **Reinforcement Learning**       | **Embodied Operation**       | **Facial Interaction**       | **Remote Control Operation**       |
|-----------------------|--------------------------|----------------------------------|-----------------------------|------------------------------|------------------------------------|
| 1 head,1 at the waist,Single Arm 4 + 1,Gripper5 reps per leg| Compatible with Raspberry Pi, Odroid, Coolpi, Chips such as RDK Sweet Potato | Open-source reinforcement learning algorithms support standing, stable walking, and accurate URDF, enabling Zero-Shot transfer| Can run upper limb operation algorithms and open-source typical small models such as ACT and DP | The robot's face uses ESP32, which can run the AI Xiaozhi software to implement facial expressions and voice Agent conversations | The robot's face uses ESP32, which can run the AI Xiaozhi software to implement facial expressions and voice Agent conversations  | Provides a control device for an isomorphic dual-arm robot, enabling data acquisition of the robot's dual arms |


The BOM of the Reinforcement Kid is mainly composed of 3D printing materials and carbon fiber sheets, which facilitates DIY and processing:
<td> <img src="https://github.com/golaced/RL_Boy/blob/master/RL_Boy_image/1.png"></td>

## BOM List
### Components and Prices (RMB):

| Project               | Attribute               | Link                                                                 | Unit Price |
|-----------------------|-------------------------|----------------------------------------------------------------------|------------|
| Router                | Black 1 Network Port + PFC Antenna | [Link](https://item.taobao.com/item.htm?id=636099840850)            | 82         |
| Handle                | 100mm Spacing Curved    | [Link](https://detail.tmall.com/item.htm?id=649810911512)           | 12.5       |
| Battery               | 6s1865: 24v7ah Rectangle / 38-70-110mm | [Link](https://item.taobao.com/item.htm?id=620559361552)            | 199        |
| DCDC (12V)            | 12V                     | [Link](https://item.taobao.com/item.htm?id=601763884843)            | 42         |
| DCDC (5V)             | 5V                      | [Link](https://item.taobao.com/item.htm?id=601763884843)            | 42         |
| 6006 Motor            | 4                       | [Link](https://item.taobao.com/item.htm?id=816922353459)            | 799        |
| 8006 Motor            | 6                       | [Link](https://item.taobao.com/item.htm?id=815097170248)            | 1009       |
| 3507 Motor            | 8                       | Consult Damiao                                                     | 300        |
| Robot Main Controller | 1 piece                 | [Link](https://item.taobao.com/item.htm?id=643604114830)            | 899        |
| Jumper Conversion Module | USB 2.0 Female Socket to DIP | [Link](https://detail.tmall.com/item.htm?id=520458685405)           | 15         |
| Power Distribution Board | 1 piece                 | See below for details                                              | 200        |
| Carbon Fiber Cutting  | 1 set                   | See below for details                                              | Inquiry    |
| Aluminum alloy processing | 1 set                   | See below for details                                              | Inquiry    |
| 3D Printing          | 1 set                   | See below for details                                              | Inquiry    |
| TF Card              | 1 x 64G                 | [Link](https://detail.tmall.com/item.htm?id=44499258098)            | 32         |
| USB Camera           | 2 pieces                | [Link](https://item.taobao.com/item.htm?id=647516320352)            | 134        |
| JetsonNano           | Single main board + TF card (Optional) | [Link](https://detail.tmall.com/item.htm?id=608609593274)          | 1119       |
| IMU                  | 1                       | [Link](https://item.taobao.com/item.htm?id=852758654977)            | 164        |
| High-performance IMU | 1                       | [Link](https://detail.tmall.com/item.htm?id=671044026458)           | 900        |
| IMU Serial Port Conversion Cable | 1                       | [Link](https://item.taobao.com/item.htm?id=647516320352)            | 40         |
| OLED                 | 1                       | [Link](https://item.taobao.com/item.htm?id=567986384172)            | 32         |
| Network Cable        | 3 (Length: 0.15m)       | [Link](https://item.taobao.com/item.htm?id=744136764912)            | 6.8        |
| Remote Control 1     | 1 piece (Bat 2 Black [High-end Model] with Vibration) | [Link](https://detail.tmall.com/item.htm?id=38896089959)           | 77         |
| Remote Control 2     | 1 piece (ESP32 OmniTele) | Produced according to the materials                                | 600        |
| Arm Cuff            | Black Large (Adult Plus Size) [(33cm long x 20cm wide)] 2 pieces | [Link](https://item.taobao.com/item.htm?id=683955453468)           | 13         |
| Foot Cushioning      | 15 thick and 40 wide    | [Link](https://detail.tmall.com/item.htm?id=740214426088)           | 11         |
| Air Switch          | Single 32A              | [Link](https://buyertrade.taobao.com/trade/detail/tradeSnap.htm?tradeID=1931951103334788577) | 31         |
| Network Port Conversion | Shielded D-Type Straight Through Socket (1 Piece) | [Link](https://detail.tmall.com/item.htm?id=674427182363)          | 20.8       |
| Step-down Module    | 5V/12V                  | [Link](https://item.taobao.com/item.htm?id=544179516749)            | 23         |
| Brushless Servo     | 2 pieces (SCS40 Optional) | [Link](https://item.taobao.com/item.htm?id=561810277224)            | 386        |
| Steering Gear SCS125 | 1 piece (Upgradable to 40 to improve head flexibility) (Optional) | [Link](https://item.taobao.com/item.htm?id=648615972680)           | 85         |
| Steering Gear Driver Board (Optional) | 1                       | [Link](https://item.taobao.com/item.htm?id=560546097733)            | 45         |

### Screws and Nuts

| Screw Type       | Link                                                                 | Number | Usefulness                          |
|------------------|----------------------------------------------------------------------|--------|-------------------------------------|
| M3*6            | [Link](https://detail.tmall.com/item.htm?id=635430936636)            | 20     | Top motor installation              |
| M3*6            | Same as above                                                       | 10     | Roll motor fixed aluminum alloy     |
| M4*8            | Same as above                                                       | 10     | Roll motor fixed aluminum alloy     |
| M4*8            | Same as above                                                       | 10     | Thigh aluminum alloy roll motor     |
| M4*8            | Same as above                                                       | 10     | Thigh aluminum alloy thigh motor    |
| M3*6            | Same as above                                                       | 28     | Thigh Carbon Plate                  |
| M3*10           | Same as above                                                       | 6      | Thigh Carbon Plate and Support      |
| M4*6            | Same as above                                                       | 10     | Lower leg carbon plate, lower leg motor |
| M3*10           | Same as above                                                       | 8      | Shin Carbon Plate and Support       |
| M3*6            | Same as above                                                       | 10     | Ankle Motor and Carbon Plate        |
| M3*8            | Same as above                                                       | 12     | Ankle Motor and Foot End            |
| M3*15           | Same as above                                                       | 8      | Upper and lower frames and supports |
| M3*10           | Same as above                                                       | 6      | Battery Compartment                 |
| M5*15           | Same as above                                                       | 4      | Bottom Support                      |
| M3*12           | Same as above                                                       | 2      | Battery top fixing                  |
| M3*10           | Same as above                                                       | 4      | Battery side fixed                  |
| M3*6            | Same as above                                                       | 4      | Main control carbon plate support   |
| M3*8            | Same as above                                                       | 4      | Power supply board fixed            |
| M2.5 Aluminum Column * 12 | [Link](https://item.taobao.com/item.htm?id=521616041424) | 4      | Main Control Carrier Board Support  |
| M2.5*5          | [Link](https://item.taobao.com/item.htm?id=521623593151)            | 4      | Main Control Carrier Board          |
| M2.5*10         | Same as above                                                       | 4      | Bottom of the main control carrier board |
| M3*12           | Same as above                                                       | 4      | JetsonNano                          |
| M3*10           | Same as above                                                       | 8      | Router Servo Driver                 |
| M4*15           | Same as above                                                       | 10     | Head servo fixed                    |
| M3*8            | Same as above                                                       | 15     | Head Servo                          |
| M3 nut self-locking | [Link](https://item.taobao.com/item.htm?id=9558815975)              | Summation | -                                  |

### Cables

| Type               | Link                                                                 | Number |
|--------------------|----------------------------------------------------------------------|--------|
| USB Cable 5CM      | [Link](https://item.taobao.com/item.htm?id=641806749737)            | 1      |
| XT-CAN Line       | [Link](https://item.taobao.com/item.htm?id=761445347736)            | 4      |
| Red and Black Power Extension Cord 16AW | [Link](https://item.taobao.com/item.htm?id=597502950509)            | 2 meters |
| XT30 Female Connector 16AW | [Link](https://item.taobao.com/item.htm?id=598407253999)            | 8      |
| XT30 Male Connector 16AW | [Link](https://item.taobao.com/item.htm?id=598407253999)            | 8      |
| 4P 2.0 Line       | [Link](https://item.taobao.com/item.htm?id=520551443113)            | 6      |
| 4P 1.25 wire 20cm | [Link](https://item.taobao.com/item.htm?id=588775918268)            | 4      |
| Double-ended 10cm 4p Coplanar | [Link](https://detail.tmall.com/item.htm?id=550579148901)          | 2      |
| Double-ended 10cm 4p Off-plane | [Link](https://item.taobao.com/item.htm?id=588775918268)           | 2      |
| DC Head 0.1m       | [Link](https://detail.tmall.com/item.htm?id=598354258377)           | 2      |

 <td> <img src="https://github.com/golaced/RL_Boy/blob/master/RL_Boy_image/2.png?raw=true" ></td>
 
## Software Version Update

### Method for updating firmware to V1.0:
1. Updated the new URDF to be consistent with the actual robot in fidelity.
2. Optimized the training software architecture.
3. Updated the hardware design and added different head solutions.
4. Embedded software updated to OmniRobLabV1.0.

### Configuration Instructions:
1. **Router Name:** Tinker-2.4G-ID Number, **Password:** 11111111.
2. **HMI Host Computer Configuration for STM32 Board:** Connect the Core and Extcan circuit boards via USB, complete the motor reverse checkbox and motor type selection according to the configuration diagram, and each motor completes CAN_ID configuration through its own Dami host computer.

#### Motor Configuration Tables:

**Left Leg (Type: STM32-Core CAN1)**

| Brushless Motor | Type    | CAN ID Number |
|-----------------|---------|---------------|
| Deflection      | 6006    | 1             |
| Side Exhibition | 8006    | 2             |
| Thigh           | 8006    | 3             |
| Calf            | 8006    | 4             |
| Sole            | 6006    | 5             |
| Waist           | 6006    | 6             |
| Head Heading    | 3507    | 7             |

**Right Leg (Type: STM32-Core CAN2)**

| Brushless Motor | Type    | CAN ID Number |
|-----------------|---------|---------------|
| Deflection      | 6006    | 1             |
| Side Exhibition | 8006    | 2             |
| Thigh           | 8006    | 3             |
| Calf            | 8006    | 4             |
| Sole            | 6006    | 5             |

**Left Arm (Type: STM32-Core CAN1)**

| Brushless Motor | Type    | CAN ID Number |
|-----------------|---------|---------------|
| Motor 1         | 3507    | 1             |
| Motor 2         | 3507    | 2             |
| Motor 3         | 3507    | 3             |
| Motor 4         | 3507    | 4             |

**Right Arm (Type: STM32-Core CAN2)**

| Brushless Motor | Type    | CAN ID Number |
|-----------------|---------|---------------|
| Motor 1         | 3507    | 1             |
| Motor 2         | 3507    | 2             |
| Motor 3         | 3507    | 3             |
| Motor 4         | 3507    | 4             |

Configure the core STM32 of the HMI interface as follows:

 <td> <img src="https://github.com/golaced/RL_Boy/blob/master/RL_Boy_image/3.png?raw=true" ></td>

Configure the core ExtCan of the HMI interface as follows:
  
 <td> <img src="https://github.com/golaced/RL_Boy/blob/master/RL_Boy_image/4.png?raw=true" ></td>

3.Update STM32 firmware:
2025/7/14

4.Update the firmware and parameters of the ODroid control software package: Update the software of the Odroid internal system via WinSCP
5.Modify rc.local to ensure the correct operation of the self-starting script:

bash
sleep 5
sudo /home/odroid/Tinker/hardware_task_tinker2 &
sleep 1
sudo /home/odroid/Tinker/mission_task &
sleep 1
sudo /home/odroid/Tinker/navigation_task_tinker2 &
sleep 1

Hardware Wiring Diagram:
 <td> <img src="https://github.com/golaced/RL_Boy/blob/master/RL_Boy_image/5.png?raw=true" ></td>



