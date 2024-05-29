# #Move-Hackathon2024

## 1. Dev Environment Setup

**Prerequisites:** Laptop or PC with *NVIDIA* graphics card

### 1.1. Tested: Windows 10/11 WSL2 Ubuntu-20.04

- [[Windows] CARLA_0.9.13.zip](https://carla-releases.s3.us-east-005.backblazeb2.com/Windows/CARLA_0.9.13.zip)
- Docker Desktop ( tested at v4.27.2 )
- Docker Compose ( tested at v2.24.5 )

### 1.2. Untested: Ubuntu 20.04.6 LTS (Focal Fossa)

> **Note:** To be verified

## 2. Run the Challenge

First, you'll need to pull the base image:

```
docker pull osrf/ros:noetic-desktop-focal
```

Next, clone this Git repository and `cd` into the folder, then

```
docker compose build
```

Now, on your host, you need to launch the CARLA simulator. Inside the downloaded package you should find an executable called `CarlaUE4.exe`:

```
CarlaUE4.exe
```

> **Note:** Launch `CarlaUE4.sh` instead if you're on Ubuntu.

You should see some nice rendering by now. Go on, have fun navigating around by your mouse and keyboard (`W`, `A`, `S`, `D`, `Q`, `E`).

![CarlaUE4 simulator started](./doc/images/carla_started.png)

Now, go back to your terminal at this repo, simply call:

```
docker compose up
```

and you shall see the challenge running.

![Game started](./doc/images/pygame_started.png)

> **Note:** If you don't need the first CARLA simulator window, you can run `CarlaUE4.exe -RenderOffScreen` instead, but then don't forget to kill it with Task Manager later.

To gracefully stop and remove all docker containers, open another terminal at this repo and:

```
docker compose down
```

> **Note:** It is also recommended to kill CARLA server everytime for a clean start.