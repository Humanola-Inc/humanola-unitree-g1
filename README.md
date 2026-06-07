# A. Prerequisites
First, install prerequisites: 
```sh
sudo apt install -y \
  ffmpeg \ 
  libsm6 \
  libxext6 \
  libstdc++6 \
  ca-certificates
```
Then, install `CYCLONEDDS`
```sh
cd /opt
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd /opt/cyclonedds
cmake -DCMAKE_INSTALL_PREFIX=/opt/cyclonedds -Bbuild .
/opt/cyclonedds/build
cmake --build . --target install --config Release
```

# B. Python Dependencies
In general, we recommend using `conda` / `mamba` or any flavours that supports downloading from `conda-forge` to install dependencies. 
```
conda create --name piper
```
then activate: 
```
conda activate piper
```
Finally, install dependencies: 
- Install `unitreesdk2py`
```sh
cd /opt
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd /opt/unitree_sdk2_python
export CYCLONEDDS_HOME="/opt/cyclonedds"
pip install --compile .
cp -r unitree_sdk2py/utils/lib /usr/local/lib/python3.11/site-packages/unitree_sdk2py/utils
rm /usr/local/lib/python3.11/site-packages/unitree_sdk2py/__init__.py
```
replace `python3.11` above with your python version.
```sh
pip install -r requirements.txt
conda install pinocchio -y
```
## C. Running the Script
Finally, run the humanola script after getting your `ROBO_ID` and `API_KEY`.
```
python3 main.py
```