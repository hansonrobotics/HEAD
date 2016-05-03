# setup kernel
sudo apt-get update
sudo apt-get install -y gcc make linux-generic

# install X server and OpenGL tools
sudo apt-get install -y xserver-xorg mesa-utils

# disable Nouveau
sudo echo -e "blacklist nouveau\noptions nouveau modeset=0" > /etc/modprobe.d/disable-nouveau.conf
sudo update-initramfs -u

# reboot to get kernel update and nouveau disabling activated
sudo reboot now

# install NVIDIA drivers

# version was chosen because was "Latest Long Lived Branch version", which sounds stable
# resources:
#   http://docs.aws.amazon.com/AWSEC2/latest/UserGuide/using_cluster_computing.html
#   http://www.nvidia.com/object/unix.html
#   http://us.download.nvidia.com/XFree86/Linux-x86_64/340.46/README/index.html

wget http://us.download.nvidia.com/XFree86/Linux-x86_64/340.46/NVIDIA-Linux-x86_64-340.46.run
sudo /bin/bash ./NVIDIA-Linux-x86_64-340.46.run --accept-license --no-questions --ui=none
sudo reboot now

# setup xorg.conf
# via https://stackoverflow.com/questions/19856192/run-opengl-on-aws-gpu-instances-with-centos
sudo apt-get install -y pkg-config
sudo nvidia-xconfig -a --use-display-device=None --virtual=1280x1024

# add missing BusID
sudo sed -i 's/    BoardName      "GRID K520"/    BoardName      "GRID K520"\n    BusID          "0:3:0"/g' /etc/X11/xorg.conf


# run x server and glxgears as test application
sudo /usr/bin/X &
DISPLAY=:0 glxgears

# ubuntu@ip-10-18-27-40:~$ DISPLAY=:0 glxgears
# 100639 frames in 5.0 seconds = 20127.781 FPS
# 101568 frames in 5.0 seconds = 20313.600 FPS
# Congrats!

