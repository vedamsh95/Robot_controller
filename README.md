# SDIR 2020 Controller
This code repository consists of two parts: the ctrl and the sim part. The ctrl folder contains all necessary code files for your controler. The sim folder contains the [V-REP](https://www.coppeliarobotics.com/) .ttt file with all necessary implementation for your simulation.
For the implementation, you are free to add any additional files. Please, however, do not change the given interfaces without highlight the changes to the course supervisors. This is, because we will come back on your generated libraries for testing and evaluation purposes. In case you change the interfaces, automated testing might not work propper.

## Prerequisites
- CLion
- On Linux: [Docker](https://docs.docker.com/engine/install/) (ideally [with WSL 2](https://docs.docker.com/docker-for-windows/wsl/) on Windows)
- On Windows: V-REP & a correctly set up MinGW toolchain in CLion

## Build with Docker

```bash
env UID=$UID docker-compose up -d --build
```
