# MATRiX

<div align="center">
  <a href="#">
    <img alt="Forest" src="demo_gif/Forest.png" width="800" height="450"/>
  </a>
</div>

<div align="center">

[![English](https://img.shields.io/badge/Language-English-blue)](README.md)
[![中文](https://img.shields.io/badge/语言-中文-red)](docs/README_CN.md)

</div>

> **Last Updated:** 2026-04-28

MATRiX is an advanced simulation platform that integrates **MuJoCo**, **Unreal Engine 5**, and **CARLA** to provide high-fidelity, interactive environments for quadruped robot research. Its software-in-the-loop architecture enables realistic physics, immersive visuals, and optimized sim-to-real transfer for robotics development and deployment.

## 🚀 Quick Start

### 1. Environment Dependencies
- **OS:** Ubuntu 22.04
- **GPU:** NVIDIA RTX 4060 or above (Driver >= 535)
- **Tools:** GCC/G++ ≥ C++11, CMake ≥ 3.16
- **ROS:** `ROS_humble`

### 2. Installation
```bash
# Clone
git clone https://github.com/zsibot/matrix.git
cd matrix

# Install system/runtime dependencies, including required local deb packages in deps/.
# The script configures the ROS 2 Humble apt source automatically if it is missing.
bash scripts/install_deps.sh

# Install release assets (base package, runtime assets, shared resources, and selected maps)
bash scripts/release_manager/install_chunks.sh 0.1.2

# Verify after dependencies and assets are installed
bash scripts/check_env.sh runtime
```
*`scripts/run_sim.sh` and `scripts/run_custom_urdf.sh` run runtime environment checks automatically before launch.*
*If the ROS apt source is blocked, rerun with `ROS_APT_REPO_URL=<reachable-ros2-apt-mirror> bash scripts/install_deps.sh`.*
*If your network hits aria2/wget TLS errors, rerun the chunk installer with `SKIP_ARIA2=1` to force the fallback download path.*
*Full offline package: [matrix_0.1.2.zip (Artifactory)](http://192.168.50.40:8081/artifactory/jszrsim/github/matrix_0.1.2.zip) / [Google Drive](https://drive.google.com/file/d/1d4q28AgSwmfv7x07oE-YF8xVOdSva9ll/view?usp=drive_link) / [Baidu Netdisk, code: `jbk3`](https://pan.baidu.com/s/12k5XJwD53ax3we3_1Gulmw?pwd=jbk3).*
*See [Chunk Packages Guide](docs/CHUNK_PACKAGES_GUIDE.md) for offline/manual installation.*

### 3. Run Simulation
```bash
./bin/sim_launcher
```
*(Select your robot and map in the launcher interface.)*

<div align="center">
  <img src="demo_gif/Launcher.png" alt="Simulation Running Example" width="640" height="360"/>
</div>

## 📚 Documentation Directory

To keep this README concise, detailed guides have been organized into the `docs/` folder:

**Basics & Setup**
- [📦 Chunk Packages Guide](docs/CHUNK_PACKAGES_GUIDE.md) - Modular package deployment & offline install
- [🎮 Controller Guide](docs/Controller_Guide.md) - Gamepad & Keyboard control mappings
- [🛠️ Scripts Guide](docs/Scripts_Guide.md) - Detailed CLI scripts usage

**Simulation & Customization**
- [🤖 Robot Types & Maps](docs/Robots_and_Maps.md) - IDs and visual previews of all robots and maps
- [⚙️ Sensor Configuration](docs/Sensor_Config_Tutorial.md) - Adjusting cameras, LiDAR, and RViz visualization
- [🌍 Custom Scene Guide](docs/Custom_Scene_Tutorial.md) - Building custom JSON-based environments
- [🐕 Custom Robot Tutorial](docs/Custom_Robot_Tutorial.md) - Importing your own MuJoCo URDF models
- [Matrix Overworld Adjacent V1](docs/OVERWORLD_ADJACENT_V1_CN.md) - Six-scene SONIC physics composition and its explicit UE visual gate
- [Matrix SONIC Video Capture](docs/MATRIX_VIDEO_CAPTURE_CN.md) - Native-resolution main-launcher recording with provenance and quality gates
- [Matrix Urban V1](docs/URBAN_V1_CN.md) - Native Town10 city baseline and URDF-preserved G1 materials

**Advanced Features**
- [🌐 Multi-Robot Tutorial](docs/Multi_Robot_Tutorial.md) - Simulating multiple robots simultaneously
- [🐳 Docker Tutorial](docs/Docker_Tutorial.md) - Running MATRiX in a container
- [📡 RoamerX Open Integration](docs/RoamerX_Lite_Integration.md) - ROS2 Nav2 stack integration
- [🎥 Pixel Streaming](docs/pixelstreaming_tutorial.md) - Web browser streaming

## 💬 Community

**Add the GENISOM AI WeChat assistant for MATRiX simulation discussions and support:**

<div align="center">
  <img src="demo_gif/wechat.png" alt="GENISOM AI WeChat Assistant QR Code" style="height: 320px; width: auto; margin: 0 12px;"/>
  <p><em>Scan to add XinQi Robo; mention MATRiX to join the simulation community.</em></p>
</div>

## 🙏 Acknowledgements

This project builds upon the incredible work of the following open-source projects:

- [MuJoCo-Unreal-Engine-Plugin](https://github.com/oneclicklabs/MuJoCo-Unreal-Engine-Plugin)
- [MuJoCo](https://github.com/google-deepmind/mujoco)
- [Unreal Engine](https://github.com/EpicGames/UnrealEngine)
- [CARLA](https://carla.org/)
