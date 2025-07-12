# shell.nix
{ pkgs ? import <nixpkgs> {} }:

let
  kernel = pkgs.linuxPackages_6_15.kernel;
in
pkgs.mkShell {
  name = "ipu6-driver-dev";

  nativeBuildInputs = with pkgs; [
    # Kernel-Entwicklung. 'kernel.dev' enthält die Header und das Build-System.
    kernel.dev

    # Standard-Build-Werkzeuge
    gcc
    gnumake
    
    # Werkzeuge zur Patch-Erstellung und Code-Analyse
    git
    diffutils
    patchutils
    ripgrep
    tree
    bat
  ];

  shellHook = ''
    clear
    echo "✅ Willkommen in der ipu6-Entwicklungsumgebung!"
    echo "   Kernel-Version: ${kernel.modDirVersion}"
    echo "--------------------------------------------------"

    export KERNEL_SRC_ORIGINAL="${kernel.src}"
    export KERNEL_DIR="${kernel.dev}/lib/modules/${kernel.modDirVersion}/build"

    # Prüfen, ob die Sourcen schon kopiert wurden
    if [ ! -d "ipu6-src" ]; then
      echo "ℹ️  Der Quellcode für den ipu6-Treiber wurde noch nicht kopiert."
      echo "    Da die Kernel-Sourcen ein .tar.xz-Archiv sind, müssen wir sie entpacken."
      echo "    Führe die folgenden Befehle aus, um nur den benötigten Ordner zu extrahieren:"
      echo
      echo "    # 1. Erstelle das Zielverzeichnis"
      echo "    mkdir ./ipu6-src"
      echo
      echo "    # 2. Entpacke nur den Intel-Treiber-Ordner dorthin"
      echo "    tar -xf ${kernel.src} --strip-components=4 -C ./ipu6-src linux-${kernel.version}/drivers/media/pci/intel"
      echo
    else
      echo "✅ Lokale Kopie des ipu6-Quellcodes in './ipu6-src' gefunden."
      echo "   Du kannst jetzt die Dateien bearbeiten und das Modul bauen."
      echo
      echo "   Zum Bauen, wechsle ins Verzeichnis und nutze 'make':"
      echo "   cd ipu6-src && make"
      echo
    fi
    
    # Hilfreiche Funktion, um einen Patch aus deinen Änderungen zu erstellen
    create_patch() {
      if [ ! -d "ipu6-src" ]; then
        echo "Fehler: Das Verzeichnis 'ipu6-src' existiert nicht."
        return 1
      fi
      echo "Erstelle Patch aus 'ipu6-src' nach './ipu-bridge.patch'..."
      (
        cd ipu6-src
        if [ ! -d ".git" ]; then
          git init > /dev/null
          git add . > /dev/null
          git commit -m "Initial ipu6 source" > /dev/null
          echo "Git-Repository zur Änderunogsverfolgung in 'ipu6-src' initialisiert."
        fi
        cp ../ipu-bridge.c ./intel/ipu-bridge.c
        git diff > ../ipu-bridge.patch
      )
      echo "Fertig! Patch-Datei wurde erstellt."
    }
  '';
}
