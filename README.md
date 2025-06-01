This is a minimal MuJoCo (ver 3.3.3) C++ template with double pendulum model

# Installation
You have two options for install & build this repository.
1. Set up and install prerequisites & dependencies in you local PC.
2. Use Docker

## To install all the requirements locally by yourself...
### Prerequisities
- OS: **Ubuntu 20.04** (or higher distro should be fine)
  - You may try with **WSL2 in Windows**, but it is not recommended for rendering simulation.
  - this repo would **not be supported for macOS**
- build tools: basic tools for build and manage C++ project are required e.g. git, gcc, cmake, ...
  ```bash
  $ sudo apt update && sudo apt upgrade -y

  $ sudo apt install build-essential
  $ gcc --version # check if gcc is installed correctly

  $ sudo apt-get install git
  $ git --version

  $ sudo apt install cmake
  $ cmake --version
  ```

- MuJoCo
  ```bash
  # Clone and build MuJoCo from source
  $ cd ~ # change path to your home directory
  $ git clone https://github.com/deepmind/mujoco.git
  $ cd ~/mujoco
  $ mkdir -p install build
  $ cd build
  $ cmake ..
  $ cmake .. -DCMAKE_INSTALL_PREFIX=~/mujoco/install
  $ cmake --build .
  $ cmake --install .

  # Add environment variables to ~/.bashrc
  $ echo 'export MUJOCO_PATH=$HOME/mujoco/install' >> ~/.bashrc
  $ echo 'export LD_LIBRARY_PATH=$MUJOCO_PATH/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
  $ source ~/.bashrc
  ```

- Dependencies
  - Eigen
  ```bash
  $ sudo apt install libeigen3-dev # Then, eigen headers are installed at /usr/include/eigen3
  ```
- Editor like **Visual Studio Code (VSCode)** or **Cursor AI** is recommended
  - Recommended Extensions
    - Better Comments
    - C/C++, C/C++ Extension Pack
    - CMake, CMake Tools
    - Clang-Format
    - Git Graph
    - Markdown All in One
    - Material Icon Theme
    - Rainbow Brackets
    - XML Tools

## Using Docker
Whether you are programming in Linux (e.g. Ubuntu) or WSL2, you can install all the prerequisites
and dependencies easily using Docker. All you have to do is to install [Docker](https://www.docker.com/)
according to your system. Though, pretty sure that you can not try this repo using Docker in macOS
due to OpenGL rendering. (If you find a solution, please let me know. I would be appreciate for that.)

```bash
# Clone the repository
$ cd ~/<your-project-directory>
$ git clone https://github.com/jwhong1209/mujoco_cpp_template.git
$ cd mujoco_cpp_template

# Build and run Docker containter
$ docker build -t mujoco-cpp-template .
$ docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $(pwd):/workspace \
    mujoco-cpp-template
```

# Build & Run
Once you installed all the requirements whether in Linux or WSL2, now you can build and run this 
examplary repo with following commands in terminal.
```bash
$ cd ~/<your-project-directory>
$ mkdir build && cd build
$ cmake ..
$ make
$ ./mujoco_simulation # or you can change the name of executable in CMakeLists.txt
```

Now you are ready for writing your own program for MuJoCo simulation :)

# Troubleshooting
If you encountered the following issue while `docker build`, this is mostly due to network problem.
```bash
error: RPC failed; curl 56 GnuTLS recv error (-9): Error decoding the received TLS packet.
fatal: the remote end hung up unexpectedly
fatal: early EOF
fatal: index-pack failed
```

Check your network and try `docker build` again.