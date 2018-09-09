To compile:

g++ -std=c++11 mycppfile.cpp -I/usr/include/flycapture -lflycapture `pkg-config --libs --cflags opencv` -o mycppfile

To enable Jumbo Packets and ensure no lag of Point Gray stream :

sudo sysctl –w net.core.rmem_max=33554432
sudo sysctl –w net.core.wmem_max=33554432
sudo sysctl –w net.core.rmem_default=33554432
sudo sysctl –w net.core.wmem_default=33554432

Note: In order for these changes to persist after system reboots, the following lines must be manually added to the bottom of the /etc/sysctl.conf file:
net.core.rmem_max=33554432
net.core.rmem_default=33554432
net.core.wmem_max=33554432
net.core.wmem_default=33554432
