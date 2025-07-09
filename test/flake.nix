{
  description = "OV01A1B Kernel Module Development Environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        kernel = pkgs.linuxPackages_6_15.kernel;
        
        # Helper function to create kernel module
        mkKernelModule = { name, src ? ./., extraBuildInputs ? [] }:
          pkgs.stdenv.mkDerivation {
            inherit name src;
            
            nativeBuildInputs = kernel.moduleBuildDependencies ++ extraBuildInputs;
            
            hardeningDisable = [ "pic" "pie" ];
            
            buildPhase = ''
              runHook preBuild
             
              # Build directly with kernel build system
              make -C ${kernel.dev}/lib/modules/${kernel.modDirVersion}/build \
                M=$(pwd) \
                KERNELRELEASE=${kernel.modDirVersion} \
                modules
              
              runHook postBuild
            '';
            
            installPhase = ''
              runHook preInstall
              
              mkdir -p $out/lib/modules/${kernel.modDirVersion}/kernel/drivers/media/i2c
              
              # Install all .ko files
              find . -name "*.ko" -exec cp {} $out/lib/modules/${kernel.modDirVersion}/kernel/drivers/media/i2c/ \;
              
              runHook postInstall
            '';
            
            meta = with pkgs.lib; {
              description = "OV01A1B camera sensor kernel module";
              license = licenses.gpl2;
              platforms = platforms.linux;
            };
          };
      in
      {
        packages = {
          # Main OV01A1B module package
          ov01a1b = mkKernelModule {
            name = "ov01a1b-module";
          };
          
          # Power test module (if you have a separate test module)
          ov01a1b-power-test = mkKernelModule {
            name = "ov01a1b-power-test";
          };
          
          # Analysis tools package
          analysis-tools = pkgs.stdenv.mkDerivation {
            name = "ov01a1b-analysis-tools";
            src = ./.;
            
            installPhase = ''
              mkdir -p $out/bin
              
              # Copy analysis scripts
              cp probe_sensor.sh $out/bin/probe-sensor
              
              chmod +x $out/bin/*
            '';
          };
        };
        
        # Default package
        defaultPackage = self.packages.${system}.ov01a1b;
        
        # Development shell
        devShells.default = pkgs.mkShell {
          name = "ov01a1b-dev-shell";
          
          buildInputs = with pkgs; [
            # Kernel development
            kernel.dev
            gnumake
            gcc
            
            # System tools
            acpica-tools 
            gpio-utils
            i2c-tools
            usbutils
            pciutils
            
            # Development tools
            gdb
            strace
            
            # Source analysis
            ripgrep
            fd
            bat
            tree
            jq
            
            # Git and diff tools
            git
            diffutils
            patchutils
            
            # Documentation
            man-pages
            man-pages-posix
            
            # Kernel module tools
            kmod
            
            # Analysis tools package
            self.packages.${system}.analysis-tools
          ];
          
          shellHook = ''
            echo "🚀 OV01A1B Kernel Module Development Shell"
            echo "==========================================="
            echo ""
            echo "📦 Package versions:"
            echo "  Kernel: ${kernel.modDirVersion}"
            echo "  GCC: $(gcc --version | head -1)"
            echo ""
            echo "🔧 Build environment:"
            echo "  KERNEL_DIR: ${kernel.dev}/lib/modules/${kernel.modDirVersion}/build"
            echo "  Build command: make KERNELDIR=\$KERNEL_DIR M=\$(pwd) modules"
            echo ""
            echo "📋 Available commands:"
            echo "  show-kernel-info  - Show detailed kernel information"
            echo "  probe-sensor      - Probe I2C sensor"
            echo ""
            echo "🏗️  Development workflow:"
            echo "  1. Edit your .c files"
            echo "  3. Run 'nix build' to package"
            echo ""
            
            # Export environment variables
            export KERNEL_DIR="${kernel.dev}/lib/modules/${kernel.modDirVersion}/build"
            export KERNELRELEASE="${kernel.modDirVersion}"
            export KBUILD_OUTPUT="$KERNEL_DIR"
            
            
            show-kernel-info() {
              echo "📊 Kernel Build Information:"
              echo "  Release: $KERNELRELEASE"
              echo "  Build dir: $KERNEL_DIR"
              echo "  Architecture: $(uname -m)"
              echo "  Available configs:"
              [ -f "$KERNEL_DIR/.config" ] && echo "    .config: ✅" || echo "    .config: ❌"
              [ -f "$KERNEL_DIR/Module.symvers" ] && echo "    Module.symvers: ✅" || echo "    Module.symvers: ❌"
              echo ""
              echo "  Module search path:"
              ls -la "$KERNEL_DIR/scripts/" | head -5
            }
            
          '';
        };
        
      });
}
