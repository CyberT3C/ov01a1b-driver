# shell.nix for camera driver development
{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  name = "camera-driver-dev";
  
  buildInputs = with pkgs; [
    # Kernel development tools
    linuxPackages_latest.kernel.dev
    
    # Build essentials
    gcc
    gnumake
    pkg-config
    ncurses
    
    # Source code analysis
    cscope
    ctags
    ripgrep
    fd
    bat
    tree
    
    # Diff and patch tools
    diffutils
    patchutils
    git
    
    # Decompression tools
    xz
    gzip
    
    # Module tools
    kmod
    
    # Debug tools
    gdb
    strace
  ];
  
  shellHook = ''
    echo "Camera Driver Development Environment"
    echo "====================================="
    
    # Create working directory
    mkdir -p camera-drivers/{ov01a10,ov01a1s,ov01a1b}
    
    # Function to extract kernel module source
    extract_module() {
      local module=$1
      local outdir=$2
      
      echo "Extracting $module to $outdir..."
      
      # Find the module
      local modpath=$(find /nix/store -name "$module.ko.xz" -type f 2>/dev/null | grep -E "linux.*6\.15\.4" | head -1)
      
      if [ -n "$modpath" ]; then
        echo "Found module at: $modpath"
        # For now just copy it, we'll get actual source differently
        cp "$modpath" "$outdir/"
      else
        echo "Module $module not found"
      fi
    }
    
    # Get kernel source if available
    get_kernel_source() {
      local kernel_version=$(uname -r)
      echo "Looking for kernel source for $kernel_version..."
      
      # Try to find kernel source in nix store
      local kernel_src=$(find /nix/store -maxdepth 1 -name "*linux-$kernel_version-source" -type d 2>/dev/null | head -1)
      
      if [ -n "$kernel_src" ]; then
        echo "Found kernel source at: $kernel_src"
        export KERNEL_SRC="$kernel_src"
      else
        echo "Kernel source not found, will try to fetch from Intel IPU6 repo"
      fi
    }
    
    # Function to download Intel IPU6 driver sources
    download_ipu6_sources() {
      if [ ! -d "ipu6-drivers" ]; then
        echo "Cloning Intel IPU6 drivers repository..."
        git clone https://github.com/intel/ipu6-drivers.git
      else
        echo "IPU6 drivers already cloned, updating..."
        cd ipu6-drivers && git pull && cd ..
      fi
    }
    
    echo -e "\nSetting up environment..."
    cd debug-camera-drivers
    
    # Download sources
    download_ipu6_sources
    
    # Extract specific driver files
    if [ -d "ipu6-drivers/drivers/media/i2c" ]; then
      echo -e "\nCopying driver sources..."
      [ -f "ipu6-drivers/drivers/media/i2c/ov01a10.c" ] && cp ipu6-drivers/drivers/media/i2c/ov01a10.c ov01a10/
      [ -f "ipu6-drivers/drivers/media/i2c/ov01a1s.c" ] && cp ipu6-drivers/drivers/media/i2c/ov01a1s.c ov01a1s/
      
      echo -e "\nAvailable OV drivers:"
      ls -la ipu6-drivers/drivers/media/i2c/ov*.c 2>/dev/null | grep -E "ov0[0-9]" || echo "No OV0x drivers found"
    fi
    
    echo "Useful paths:"
    echo "  IPU6 sources: $(pwd)/ipu6-drivers"
    echo "  Driver sources: $(pwd)/ov01a*/"
    echo ""
  '';
}
