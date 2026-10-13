#!/usr/bin/env bash
#
# Usage: ./install-autostart.sh <env_name>
#   e.g. ./install-autostart.sh humanola
#
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 <env_name>   (e.g. $0 humanola)"
    exit 1
fi

ENV_NAME="$1"

SERVICE_NAME="humanola-unitree-g1"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CYCLONEDDS_HOME="$REPO_DIR/build/cyclonedds/install"

# Prerequisites
CONDA=""
for candidate in mamba micromamba conda; do
    if command -v "$candidate" >/dev/null 2>&1; then
        CONDA="$candidate"
        break
    fi
done
if [[ -z "$CONDA" ]]; then
    echo "conda/mamba/miniconda not found in PATH"
    exit 1
fi
for bin in git cmake gcc; do
    if ! command -v "$bin" >/dev/null 2>&1; then
        echo "$bin not found in PATH"
        exit 1
    fi
done

echo "fetching libs"
git -C "$REPO_DIR" submodule update --init --recursive

# Conda env
if "$CONDA" env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
    echo "conda env '$ENV_NAME' already exists, reusing it"
else
    echo "creating conda env '$ENV_NAME'"
    "$CONDA" create -y -n "$ENV_NAME" -c conda-forge python=3.11
fi

echo "installing pinocchio and casadi"
"$CONDA" install -y -n "$ENV_NAME" -c conda-forge pinocchio casadi

# CycloneDDS, unitree_sdk2py needs it to build its python bindings
echo "building cyclonedds"
cmake -S "$REPO_DIR/libs/cyclonedds" -B "$REPO_DIR/build/cyclonedds" \
    -DCMAKE_INSTALL_PREFIX="$CYCLONEDDS_HOME"
cmake --build "$REPO_DIR/build/cyclonedds" --target install --config Release

echo "installing unitree_sdk2py"
CYCLONEDDS_HOME="$CYCLONEDDS_HOME" "$CONDA" run -n "$ENV_NAME" \
    pip install "$REPO_DIR/libs/unitree_sdk2_python"
SITE_PACKAGES=$("$CONDA" run -n "$ENV_NAME" python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])")
# pip leaves out the prebuilt crc libraries
cp -r "$REPO_DIR/libs/unitree_sdk2_python/unitree_sdk2py/utils/lib" "$SITE_PACKAGES/unitree_sdk2py/utils/"
# the package __init__ eagerly imports every robot, it has to go for the
# g1 imports to work (same as the old dockerfile)
rm -f "$SITE_PACKAGES/unitree_sdk2py/__init__.py"

echo "installing inspire_demos"
"$CONDA" run -n "$ENV_NAME" pip install -e "$REPO_DIR/libs/inspire_demos"

echo "installing humanola"
"$CONDA" run -n "$ENV_NAME" pip install numpy
"$CONDA" run -n "$ENV_NAME" pip install --extra-index-url https://releases.humanola.com/py/ humanola

ENV_PYTHON=$("$CONDA" run -n "$ENV_NAME" which python3)

echo "installing $SERVICE_NAME service"
sudo tee "/etc/systemd/system/$SERVICE_NAME.service" >/dev/null <<EOF
[Unit]
Description=Humanola Unitree G1
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=root
WorkingDirectory=$REPO_DIR/src
Environment=PYTHONUNBUFFERED=1
Environment=CYCLONEDDS_HOME=$CYCLONEDDS_HOME
Environment=LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib
ExecStart=$ENV_PYTHON $REPO_DIR/src/main.py
KillSignal=SIGINT
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl restart "$SERVICE_NAME"

echo "done. logs: journalctl -u $SERVICE_NAME -f"
