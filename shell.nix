{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  packages = [
    pkgs.gnumake
    pkgs.python311
    pkgs.python311Packages.pip
    pkgs.minicom
  ];

  shellHook = ''
    if [ ! -d .venv ]; then
      python -m venv .venv
      .venv/bin/pip install -r requirements.txt
    fi
    export PIP_DISABLE_PIP_VERSION_CHECK=1
    export MINICOM='-c off'
  '';
}

