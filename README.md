# ioniq_ws
CBNU AVLab ioniq_ws for Ros

# Setup
Linux Ubuntu 16.04\n
Ros Version : Kinetic\n
Qt Version : 5.\n
Add above line to .bashrc\n
--> export LD_LIBRARY_PATH="your_qt5_path/gcc_64/lib":$LD_LIBRARY_PATH\n

# Build
catkin_make -DCATKIN_WHITELIST_PACKAGES="vehicle_msgs"\n
catkin_make
