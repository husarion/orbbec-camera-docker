<h1 align="center">
Docker Images for Orbbec cameras
</h1>

The repository contains a GitHub Actions workflow for auto-deployment of built Docker images to [husarion/orbbec-camera](https://hub.docker.com/r/husarion/orbbec-camera) DockerHub repositories.

[![ROS Docker Image](https://github.com/husarion/orbbec-camera-docker/actions/workflows/ros-docker-image.yaml/badge.svg)](https://github.com/husarion/orbbec-camera-docker/actions/workflows/ros-docker-image.yaml)

## Available images

- **`husarion/orbbec-camera:humble`**,
- **`husarion/orbbec-camera:iron`**.

## Prepare environment

1. **Plugin the camera**

For best performance please use **USB 2.0/3.0** port, depend of the camera model. Then use `lsusb` command to check if the device is visible.

2. **Select the appropriate launch file**

```bash
export CAMERA_LAUNCH=<camera_launch>
```

Replace `<camera_launch>` with appropriate launch file for your camera from below table.

| **Product name**  | **Firmware version** | **Launch file**         |
| ----------------- | -------------------- | ----------------------- |
| Astra+            | 1.0.22 - 1.0.19      | astra_adv.launch.py     |
| Astra 2           | 2.8.20               | astra2.launch.py        |
| Astra Mini Pro    | 1007                 | astra.launch.py         |
| Astra Mini S Pro  | 1.0.05               | astra.launch.py         |
| DaBai             | 2436                 | dabai.launch.py         |
| DaBai DCW         | 2460                 | dabai_dcw.launch.py     |
| DaBai DW          | 2606                 | dabai_dw.launch.py      |
| Femto             | 1.6.7                | femto.launch.py         |
| Femto Mega        | 1.1.7                | femto_mega.launch.py    |
| Femto W           | 1.1.8                | femto.launch.py         |
| Gemini            | 3.0.18               | gemini.launch.py        |
| Gemini 2          | 1.4.60 /1.4.76       | gemini2.launch.py       |
| Gemini 2 L        | 1.4.32               | gemini2L.launch.py      |
| Gemini E          | 3460                 | gemini_e.launch.py      |
| Gemini E Lite     | 3606                 | gemini_e_lite.launch.py |

## Demo

1. **Clone repository**

```bash
git clone https://github.com/husarion/orbbec-camera-docker.git
cd orbbec-camera-docker/demo
```

2. **Run `compose.yaml`**

```bash
xhost local:root
docker compose up
```