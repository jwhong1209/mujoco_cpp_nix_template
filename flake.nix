{
  description = "Minimal MuJoCo Simulation Template in Nix";

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

              # other pkgs
              pkgs.pinocchio
              pkgs.casadi
            ];
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
