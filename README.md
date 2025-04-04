# OpenArm: an open-source robotic arm for human manipulation data collection
OpenArm is an open-source humanoid robot arm designed for AI and robotics research in human-centered environments. Its modular hardware and accessible software make it a flexible platform for teleoperation, imitation learning, and real-world data collection. Our goal is to advance physical intelligence by enabling robots that operate safely and effectively alongside people — in homes, service contexts, and caretaking settings. OpenArm offers high backdrivability and compliance, making it well-suited for interactive, assistive, and data-driven tasks.

The project is under active development, and we’re collaborating with researchers, developers, and labs to shape the next generation of practical humanoid systems.

![Group 177](https://github.com/user-attachments/assets/033b1c4d-2b5a-43d4-ac3c-0cde2157ff43)


## 🚀 OpenArm v0.2 (beta) Updates:
- **Gravity Compensation:** Smoother teleoperation with real-time compensation.
- **Force Feedback Teleoperation:** Unilateral and **bilateral control** with force feedback for better manipulation and data collection.
- **URDF Overhaul:** Improved default pose and model accuracy.

### ▶️ Watch OpenArm v0.2 in Action:

[<img src="https://github.com/user-attachments/assets/9d920921-383c-4d1a-980c-0412a1b59957" alt="openarm thumb 1" width="450">](https://www.youtube.com/watch?v=2-Au7Sc0uKw?si=RIR9v3v0valV4106 "OpenArm v0.2 beta")

## Table of Contents:
* [**Quick Start**](#Quick-Start)
* [**About**](#About)
* [**Hardware**](#Hardware-🛠️)
* [**Software**](#Software-💾)
  * [**Arm Control**](#Arm-Control-🦾)
  * [**ROS2 Packages**](#ROS2-Packages-📦)
  * [**Simulation**](#Simulation-🤖)
  * [**Teleoperation**](#Teleoperation-🏗️)
* [**Contact Us**](#Contact-Us)

## Quick Start

```sh
git clone --recurse-submodules https://github.com/reazon-research/OpenArm.git
```

## About
This repository includes everything needed to build, simulate, and operate OpenArm — including hardware files, software, and example setups for teleoperation and simulation.

For more information and ways to get involved:
- OpenArm:
  - [**Website**](https://private-reazon.notion.site/OpenArm-113446ca7f73805fa06cd8d24315122b)
  - [**Join our Discord**](https://discord.gg/K6kmFzXagm)
- Reazon HI Lab:
  - [**Website**](https://www.hilab.jp/Reazon-Human-Interaction-Lab-113446ca7f7381a987f4f091d3f62dd5)
  - [**X (@ReazonHILab)**](https://x.com/reazonhilab)
  - [**Contact Us**](#Contact-Us)

## Hardware 🛠️
![OpenArm_Spec_Main_Graphic_WHITE](https://github.com/user-attachments/assets/33b801f6-6ee4-45a7-875e-de81dafd986b)

The CAD for the OpenArm v1 project can be accessed on OnShape through the following link: 
- [OnShape](https://cad.onshape.com/documents/b4c9f28b9b00f7d40a1a4250/w/fe370058f6ecce02af3b0093/e/c7e7f88d1c11b5ea0a83ba7c?renderMode=0&uiState=67b590ed2d89b65cc3bf2317)

Machined, sheet metal, and off-the-shelf components can be purchased through MISUMi and Misumi and its manufacturing service Meviy, but similar services worldwide can also be used to procure the necessary parts. The Purchasing List/Bill of Materials can be found on the OnShape, as well as the Assembly Guide. 

- [Risk Assessment Guideline](https://docs.google.com/spreadsheets/d/11ayqCXhusLvExf8lalkxcZMikRYgav0Hl6p7CVpsXZ8/edit?usp=sharing)

## Software 💾

### Arm Control 🦾
OpenArm’s software stack includes real-time motor control examples, enabling users to quickly set up and start moving the arm. It also provides motor calibration tools, a SocketCAN driver, and a step-by-step tutorials on setting up the CAN interface.
- [Arm Control](https://github.com/reazon-research/OpenArm/tree/main/software/control)

### ROS2 Packages 📦
Packages for camera integration, hardware bringup, and MoveIt2 can all be found under ROS2 packages. URDF descriptions for single and bimanual arm setups are also located here.
- [ROS2 Packages](https://github.com/reazon-research/openarm_ros2)

### Simulation 🤖
![image](https://github.com/user-attachments/assets/38d35919-a526-4636-9b34-b4b4ad11a32e)
To set up OpenArm in simulation environments, example documentation for MuJuCo, MoveIt2 (ROS2), and Genesis are provided.

- [Arm Simulation with MuJuCo, MoveIt2, Genesis](https://github.com/reazon-research/openarm_simulation)

### Teleoperation 🏗️
![DSCF3195](https://github.com/user-attachments/assets/6bb219fa-276f-46a6-8c31-756a8cbc19bb)

To configure one set of OpenArms to act in teleoperation, the setup is provided in the Unilateral and Bilateral links.
- COMING SOON!

## Contact Us
If you would like to get in contact with the OpenArm team with more specific questions about the project, please reach out via email and a member of the team will try to get in touch!
- Email: [hi_public@reazon.jp](hi_public@reazon.jp)

## License
OpenArm is open source under the Apache-2.0 license, see the [**LICENSE**](https://github.com/reazon-research/OpenArm/tree/main/LICENSE) file

<a href="#top">Back to top</a>
