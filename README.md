# OpenArm: an open-source robotic arm for human manipulation data collection
![Group 177](https://github.com/user-attachments/assets/033b1c4d-2b5a-43d4-ac3c-0cde2157ff43)

### 🚀 OpenArm v0.2 (beta) Highlights
[▶️ Watch Video](https://www.youtube.com/watch?v=2-Au7Sc0uKw)
- **Gravity Compensation:** Smoother teleoperation with real-time compensation.
- **Force Feedback Teleoperation:** Unilateral and **bilateral control** with force feedback for better manipulation and data collection.
- **URDF Overhaul:** Improved default pose and model accuracy.


## Table of Contents:
* [**Quick Start**](#Quick-Start)
* [**About**](#About)
* [**Hardware**](#Hardware)
* [**Software**](#Software)
  * [**Arm Control**](#Arm-Control)
  * [**Simulation**](#Simulation)
  * [**Teleoperation**](#Teleoperation)
* [**Other Resources**](#Other-Resources) 

## Quick Start

```sh
git clone --recurse-submodules https://github.com/reazon-research/OpenArm.git
```

## About

This repository outlines all the necessary materials and software required in order to build, setup, and deploy OpenArm. Examples to assist with robot simulation setup in Genesis and Gazebo environments and support for tele-operation with bilateral feedback implementation are provided.

For more background about project details and features, please visit our website, social media, and other resources:
- [**OpenArm Website**](https://private-reazon.notion.site/OpenArm-113446ca7f73805fa06cd8d24315122b)
- [**Reazon HI Lab Website**](https://www.hilab.jp/Reazon-Human-Interaction-Lab-113446ca7f7381a987f4f091d3f62dd5)
- [**Human Interaction Lab X**](https://x.com/reazonhilab)
- [**OpenArm Discord**](https://discord.gg/K6kmFzXagm)
- [**Other Resources**](#Other-Resources)

# Hardware 🛠️
![OpenArm_Spec_Main_Graphic_WHITE](https://github.com/user-attachments/assets/33b801f6-6ee4-45a7-875e-de81dafd986b)


The CAD for the OpenArm v1 project can be accessed on OnShape through the following link: 
- [OnShape](https://cad.onshape.com/documents/b4c9f28b9b00f7d40a1a4250/w/fe370058f6ecce02af3b0093/e/c7e7f88d1c11b5ea0a83ba7c?renderMode=0&uiState=67b590ed2d89b65cc3bf2317)

Machined, sheet metal, and off-the-shelf components can be purchased through MISUMi and Misumi and its manufacturing service Meviy, but similar services worldwide can also be used to procure the necessary parts. The Purchasing List/Bill of Materials can be found on the OnShape, as well as the Assembly Guide. 

# Software 💾

### Arm Control 🦾
OpenArm’s software stack includes real-time motor control examples, enabling users to quickly set up and start moving the arm. It also provides motor calibration tools, a SocketCAN driver, and a step-by-step tutorials on setting up the CAN interface.
- [Arm Control](https://github.com/reazon-research/OpenArm/tree/main/software/control)

### Simulation 🤖
![image](https://github.com/user-attachments/assets/38d35919-a526-4636-9b34-b4b4ad11a32e)
To set up OpenArm in simulation environments, example documentation for MuJuCo, MoveIt2 (ROS2), and Genesis are provided.

- [Arm Simulation with MuJuCo, MoveIt2, Genesis](https://github.com/reazon-research/openarm_simulation)
- [ROS2 Packages](https://github.com/reazon-research/openarm_ros2)

### Teleoperation 🏗️
![DSCF3195](https://github.com/user-attachments/assets/6bb219fa-276f-46a6-8c31-756a8cbc19bb)

To configure one set of OpenArms to act in teleoperation, the setup is provided in the Unilateral and Bilateral links.
- COMING SOON!

# Other Resources 📠

For more information about specifics regarding the OpenArm project, please have a look at the resources below. If you would like to get in contact with the OpenArm team with more specific questions about the project, please reach out via email and a member of the team will try to get in touch!
- [Risk Assessment Guideline](https://docs.google.com/spreadsheets/d/11ayqCXhusLvExf8lalkxcZMikRYgav0Hl6p7CVpsXZ8/edit?usp=sharing)
- Email: [hi_public@reazon.jp](hi_public@reazon.jp)

<a href="#top">Back to top</a>
