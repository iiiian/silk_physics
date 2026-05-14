let
  pkgs = import <nixpkgs> {
    config = {
      allowUnfree = true; # Required for proprietary CUDA packages
    };
  };
in
pkgs.mkShell {
  nativeBuildInputs = with pkgs; [
    gcc
    cudaPackages_13.cudatoolkit
    openblas
    ninja
    cmake       # Required to generate Ninja build files
    mold
    clang-tools # Provides clangd, clang-format, clang-tidy
    git
  ];

  shellHook = ''
    export CUDA_PATH=${pkgs.cudatoolkit}
  '';
}
