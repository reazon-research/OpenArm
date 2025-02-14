# OpenArm: A fully open-source robot arm project

This repository outlines all the necessary materials and software repositories required in order to build, setup, and deploy OpenArm. These are outlined in the [Hardware](#Hardware) and [Software](#Software) sections. In addition this, this repository also provides sample code to assist with robot simulation setup in Genesis and Gazebo environments. Finally, there is example code for tele-operation with bilateral feedback implementation. Directions the setup for these features are detailed in the [Simulation](#Simulation) and [Teleoperation](#Teleoperation) sections.

For more background about project details and features, please visit our website, social media, and other resources:
- [**Website**](https://www.notion.so/reazon-research/OpenArm-113446ca7f73805fa06cd8d24315122b)
- [**X**](https://x.com/reazonhilab)
- [**Discord**]()
- [**Other Resources**](#Other_Resources)

## 🛠️Hardware

The CAD for the OpenArm v1 project can be accessed on OnShape through the following link: 
- Provide CAD overview

With the philosophy of sourcing all components through the popular marketplace Misumi and its manufacturing service Meviy, we provide a Bill of Materials required for assembling the OpenArm v1. This is merely an example—similar services can also be used to procure the necessary components.
- [Purchasing List/BOM](https://docs.google.com/spreadsheets/d/10wI58rFytYfibKpOzmfWOQUBK5YGNwlppUtHooCTtds/edit?usp=sharing)

We provide this assembly guide as a reference for assembling the OpenArm v1. It outlines the necessary steps and components to help ensure a smooth assembly process.
- [Assembly Guide](https://docs.google.com/presentation/d/1dnI2y_ZBW3xklD3-P3SxRs0a9OQ9p0kbQuDdB3iYupA/edit?usp=sharing)
- [Pedestal]

## 💾Software
OpenArm’s software stack includes real-time motor control examples, enabling users to quickly set up and start moving the arm. It also provides motor calibration tools, a SocketCAN driver, and a step-by-step tutorials on setting up the CAN interface.
- [Arm Control](https://github.com/reazon-research/OpenArm/tree/main/software/arm_control)

## 🤖Simulation
- Genesis Tutorial/Examples
- Support for MuJuCo, MoveIt, Gazebo

## 🏗️Teleoperation
- Unilateral Control
- Bilateral Control (If implemented in time)

## 📠Other Resources
- [Risk Assessment Guideline](https://docs.google.com/spreadsheets/d/11ayqCXhusLvExf8lalkxcZMikRYgav0Hl6p7CVpsXZ8/edit?usp=sharing)
- [Contact Form]()

<a href="#top">Back to top</a>
