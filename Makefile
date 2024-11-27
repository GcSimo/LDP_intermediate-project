all:
#	compilazione con file LidarDriver.cpp unico
	g++ src/LidarDriver.cpp src/main.cpp -o build/main

#	compilazione con file LidarDriver.cpp spezzettato
#	g++ src/LidarDriver_pt1.cpp src/LidarDriver_pt2.cpp src/LidarDriver_pt3.cpp src/main.cpp -o build/main
