{
  description = "Minimal MuJoCo (C++) Simulation Template in Nix";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-25.05";
    nixgl.url = "github:nix-community/nixGL";
  };

  outputs = { self, nixpkgs, nixgl }:
    let
      system = "x86_64-linux";
      
      overlays = [
        nixgl.overlay
      ];
      
      pkgs = import nixpkgs {
        inherit system overlays;
      };
      
    in
      {
        devShells.${system} = {

          default = pkgs.mkShell {
            packages = [
              # simulation pkgs
              pkgs.mujoco
              pkgs.glfw
              pkgs.nixgl.nixGLIntel

              # dependencies
              pkgs.urdfdom
              pkgs.urdfdom-headers
              pkgs.pinocchio
              pkgs.casadi
              pkgs.eigen

              # build tools
              pkgs.cmake
              pkgs.pkg-config
            ];

            # Environment variable path setting for CMake
            shellHook = ''
              export CMAKE_PREFIX_PATH="${pkgs.urdfdom-headers}/lib/cmake:${pkgs.urdfdom}/lib/cmake:${pkgs.pinocchio}/lib/cmake:${pkgs.casadi}/lib/cmake:${pkgs.eigen}/lib/cmake:$CMAKE_PREFIX_PATH"
              export PKG_CONFIG_PATH="${pkgs.urdfdom-headers}/lib/pkgconfig:${pkgs.urdfdom}/lib/pkgconfig:$PKG_CONFIG_PATH"
              export CMAKE_MODULE_PATH="${pkgs.urdfdom-headers}/lib/cmake:${pkgs.urdfdom}/lib/cmake:$CMAKE_MODULE_PATH"
              
              export urdfdom_headers_DIR="${pkgs.urdfdom-headers}/lib/cmake/urdfdom_headers"
              
              #echo "MuJoCo development environment loaded"
              #echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"
            '';
          };

        };
      };

  nixConfig = {
    bash-prompt-prefix = "\\[\\e[38;5;12m\\](nix)\\[\\e[38;5;220m\\] ";

    extra-substituters = [
      "https://ros.cachix.org"
      "https://nix-community.cachix.org"
    ];
    
    extra-trusted-public-keys = [
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
      "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
    ];
  };
}
