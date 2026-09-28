<h1 align="center">Vision-Guided Industrial Robotic Sorting 🦾📦</h1>
<h4 align="center">Optical Polarization Adaptation, Sub-Pixel PCA Kinematics, & Vendor-Agnostic OPC UA / ROS 2 Integration</h4>

<p align="center">
  <img src="https://img.shields.io/badge/OpenCV-4.8+-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white" alt="OpenCV"/>
  <img src="https://img.shields.io/badge/C++-17-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white" alt="C++"/>
  <img src="https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python"/>
  <img src="https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white" alt="ROS 2"/>
  <img src="https://img.shields.io/badge/OPC_UA-IEC_62541-004088?style=for-the-badge" alt="OPC UA"/>
  <img src="https://img.shields.io/badge/IEEE-Review_Manuscript-00629B?style=for-the-badge&logo=ieee&logoColor=white" alt="IEEE"/>
  <img src="https://img.shields.io/badge/License-MIT-green?style=for-the-badge" alt="MIT License"/>
    <img src="https://img.shields.io/badge/IEEE-Review_Manuscript-0
</p>

<p align="center">
  <img src="https://via.placeholder.com/850x420/0a0a0a/00ffff?text=[CYBER-PHYSICAL+SORTING+CELL+&+OPTICAL+TESTBED+PIPELINE]" alt="Workstation Architecture" width="100%"/>
</p>

---

<details open>
  <summary><b>📑 DIRECTORY TERMINAL (TABLE OF CONTENTS)</b></summary>
  <ol>
    <li><a href="#overview">Executive Systems Overview</a></li>
    <li><a href="#pipeline">Cyber-Physical Dataflow Architecture</a></li>
    <li><a href="#optics">Optical Conditioning & Cross-Polarization Physics</a></li>
    <li><a href="#kinematics">Orientation Estimation & Kinematic Formulations</a></li>
    <li><a href="#calibration">Hand-Eye Calibration ($AX = XB$) & Coordinate Frames</a></li>
    <li><a href="#benchmarks">Comparative Performance & Latency Benchmarks</a></li>
    <li><a href="#middleware">Vendor-Neutral Industrial Middleware</a></li>
    <li><a href="#repository-tree">Repository Tree Architecture</a></li>
    <li><a href="#quickstart">Build & Reproduction SOP</a></li>
    <li><a href="#citation">Academic Citation</a></li>
  </ol>
</details>

---

### <a id="overview"></a>🌐 EXECUTIVE SYSTEMS OVERVIEW

<div align="justify">
High-throughput flexible manufacturing demands continuous, high-velocity workpiece inspection and pick-and-place manipulation on moving linear conveyors. However, industrial execution is constrained by two rigid operational thresholds: <b>sub-second cycle latency ($< 1\,\text{s}$)</b> and <b>tight spatial mechanical tolerances ($\le \pm 2\,\text{mm}$)</b>.

Machined metallic workpieces (e.g., spur gears, cast iron valve housings) exhibit non-Lambertian reflectance, causing intense specular glare that deteriorates sub-pixel edge detection kernels. Concurrently, deep neural networks introduce excessive edge inferencing latency, while closed-vendor robot controllers restrict extrinsic matrix interchange.

This repository provides an open-source, vendor-agnostic reference implementation of the <b>three-tier hybrid sorting architecture</b> formalized in our research review:
1. <b>Tier 1 (Optical Layer):</b> Hardware-level cross-polarization ($0^\circ / 90^\circ$) eliminating specular saturation without software HDR latency.
2. <b>Tier 2 (Kinematic Layer):</b> Real-time Principal Component Analysis (PCA) eigenvector decomposition with third-order skewness disambiguation executing in $12\text{--}28\,\text{ms}$ on CPU threads.
3. <b>Tier 3 (Middleware Layer):</b> Deterministic coordinate serialization via OPC UA (IEC 62541) and ROS 2 DDS.
</div>

---

### <a id="pipeline"></a>🔄 CYBER-PHYSICAL DATAFLOW ARCHITECTURE

```text
[ Conveyor Gantry ] ───► [ Ingress Trigger ] ───► [ Cross-Polarized Acquisition ]
                                                          │
  ┌───────────────────────────────────────────────────────┘
  ▼
[ Sub-Pixel Canny Segmentation ] ───► [ PCA Covariance Decomposition ]
                                                          │
  ┌───────────────────────────────────────────────────────┘
  ▼
[ 180° Skewness Disambiguation ] ───► [ Eye-to-Hand Kinematic Solver (AX=XB) ]
                                                          │
  ┌───────────────────────────────────────────────────────┘
  ▼
[ OPC UA / ROS 2 Middleware ] ────► [ 6-DOF Manipulator Interception Trajectory ]
