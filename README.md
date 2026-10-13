# Humanola Unitree G1
## A. Prerequisites
Install the prerequisites:
```
sudo apt install -y \
  build-essential \
  cmake \
  git \
  ffmpeg \
  libsm6 \
  libxext6 \
  libstdc++6 \
  ca-certificates
```
and a `conda` flavour that can download from `conda-forge` (`mamba`, `micromamba`, `miniforge`, ...).

## B. Configuring
Fill in your `API_KEY` and `ROBO_ID` in `src/main.py`.

## C. Installing
```
./install-autostart.sh <env_name>
```
This builds CycloneDDS, installs `unitree_sdk2py`, `inspire_demos` and `humanola` into the conda env `<env_name>`, and installs a systemd service that starts on boot. Logs:
```
journalctl -u humanola-unitree-g1 -f
```

## D. Running Without The Robot
```
conda activate <env_name>
cd src
python3 main.py --sim
```
