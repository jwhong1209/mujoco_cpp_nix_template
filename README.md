This is a minimal MuJoCo (ver 3.2.5) C++ template in Nix Environment (nixpkg ver 24.11).

# Dependencies
- MuJoCo
- Eigen 
- Pinocchio 
- CasADi (It is not utilized in codes currently, but involved in `flake.nix`)

>[!note]
> All the dependencies are configured in `flake.nix` file. You need to install Nix, and get used to how to it first.

# Quick Start
You may run this repo either in Ubuntu or WSL2 in Windows
- Ubuntu distro may not be an issue, but 20.04 or recent distro is recommended.
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

```bash
$ cd ~/<your-project-directory>
$ git clone https://github.com/jwhong1209/mujoco_cpp_nix_template.git
$ cd mujoco_cpp_nix_template
$ nix develop # setup nix environment
$ mkdir build && cd build 
$ cmake ..
$ make 
$ nixGLIntel ./simulation # run MuJoCo executable
```
