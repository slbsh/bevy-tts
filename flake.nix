{
  description = "project_name";

  inputs = {
    # nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    nixpkgs.url = "github:nixos/nixpkgs/nixos-23.11";
    systems.url = "github:nix-systems/default";
    naersk.url = "github:nix-community/naersk";
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = {
    self,
    nixpkgs,
    naersk,
    fenix,
    systems,
    ...
  }: let
    forEachSystem = nixpkgs.lib.genAttrs (import systems);
    pkgsFor = forEachSystem (system:
      import nixpkgs {
        inherit system;
        overlays = [
          fenix.overlays.default
        ];
      });
    rust-toolchain = forEachSystem (system: pkgsFor.${system}.fenix.stable);
  in {
    formatter = forEachSystem (system: pkgsFor.${system}.alejandra);

    devShells = forEachSystem (system: {
      default = pkgsFor.${system}.mkShell rec {
        packages = with rust-toolchain.${system}; [
          cargo
          rustc
          clippy
          rustfmt
        ];
        nativeBuildInputs = with (pkgsFor.${system}); [
          pkg-config
          udev alsa-lib vulkan-loader vulkan-tools
          xorg.libX11 xorg.libXcursor xorg.libXi xorg.libXrandr # To use the x11 feature
          libxkbcommon
        ];
        # buildInputs =with pkgsFor.${system};  [
        # ];
        RUSTFLAGS = "-C link-args=-Wl,-rpath,${pkgsFor.${system}.lib.makeLibraryPath [pkgsFor.${system}.libGL]}";
        LD_LIBRARY_PATH = pkgsFor.${system}.lib.makeLibraryPath nativeBuildInputs + ":/run/opengl-driver/lib:/run/opengl-driver-32/lib";
        RUST_SRC_PATH = "${rust-toolchain.${system}.rust-src}/lib/rustlib/src/rust/library";
      };
    });

    packages = forEachSystem (system: {
      default =
        (pkgsFor.${system}.callPackage naersk {
          inherit (rust-toolchain.${system}) cargo rustc;
        })
        .buildPackage {
          src = ./.;
        };
    });

    apps = forEachSystem (system: {
      default = {
        type = "app";
        program = "${self.packages.${system}.default}/bin/project_name";
      };
    });
  };
}
