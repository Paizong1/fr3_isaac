$wslCommand = @'
set -e
sudo apt-get update
sudo apt-get install -y ros-humble-cv-bridge ros-humble-vision-msgs python3-opencv python3-pip python3-venv
python3 -m venv --system-site-packages /home/paizong/.venvs/fr3-yolo
source /home/paizong/.venvs/fr3-yolo/bin/activate
python -m pip install --upgrade pip
# ROS Humble cv_bridge is compiled against NumPy 1.x; keep this venv ABI-compatible.
python -m pip install "setuptools<80" "numpy<2" "opencv-python<5" ultralytics
'@

wsl.exe -d Ubuntu-22.04 -u paizong -- bash -lc $wslCommand
