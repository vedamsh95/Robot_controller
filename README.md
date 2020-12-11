# SDIR 2020 Controler
This code repository consists of two parts: the ctrl and the sim part. The ctrl folder contains all necessary code files for your controler. The sim folder contains the [V-REP](https://www.coppeliarobotics.com/) .ttt file with all necessary implementation for your simulation.
For the implementation, you are free to add any additional files. Please, however, do not change the given interfaces without highlight the changes to the course supervisors. This is, because we will come back on your generated libraries for testing and evaluation purposes. In case you change the interfaces, automated testing might not work propper.

## Prerequisites
- Linux (or [Git Bash](https://git-scm.com/downloads) on Windows) with Git, GNU Make & wget (those are probably installed by default)
- [Docker](https://docs.docker.com/engine/install/) (ideally [with WSL 2](https://docs.docker.com/docker-for-windows/wsl/) on Windows)
- on Windows: [Xming](https://sourceforge.net/projects/xming/files/Xming/6.9.0.31/Xming-6-9-0-31-setup.exe/download)

**On Windows: all commands must be run inside the Git Bash!**

## Working with Git
A good friendly GUI for Git is [GitFiend](https://gitfiend.com/).

You will have to clone the repository first using the Git Bash though, and set the required config parameter so GitFiend can access your credentials:
```bash
git config --global credential.helper store
git clone -b group-6 --recurse-submodules https://YOUR_BITBUCKET_USERNAME@bitbucket.org/cse_admin/sdir_2020.git ~/Desktop/sdir_2020
```

You can now start GitFiend, and in the three-dot-menu in the upper left corner, uncheck "Repo" → "Use built-in Git". **Important:** after installing Git, you have to restart your computer; otherwise this step won't work!

Then, "Open a repo" and select the folder where you cloned the repository to (`sdir_2020` on the Desktop in this case).

## Build
For building your code, please use the contained [cmake](https://cmake.org/) files with Docker and the provided Makefile:

```bash
make ctrl
```

## Run
**TODO: On Windows, the X server is missing! Hence, starting the application won't work currently.**

1. Start V-REP/CoppeliaSim with the following command:
   ```bash
   make start-vrep
   ```
2. In CoppeliaSim, start the simulation by clicking the "play" button.
3. Start the controller software:
   ```bash
   make start-ctrl
   ```
