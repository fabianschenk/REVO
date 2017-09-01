# Robust Edge-based Visual Odometry (REVO)

In this work, we present a robust edge-based visual odometry (REVO) system for RGBD sensors. Edges are more stable under varying lighting conditions than raw intensity values, which leads to higher accuracy and robustness in scenes, where feature- or photoconsistency-based approaches often fail. The results show that our method performs best in terms of trajectory accuracy for most of the sequences indicating that edges are suitable for a multitude of scenes.

## If you use this work, please cite any of the following publications:
* **Combining Edge Images and Depth Maps for Robust Visual Odometry**, Schenk Fabian, Fraundorfer Friedrich, BMVC 2017, [pdf](https://pure.tugraz.at/portal/files/10383987/0661.pdf)
* **Robust Edge-based Visual Odometry using Machine-Learned Edges**, Schenk Fabian, Fraundorfer Friedrich, IROS 2017

## License
REVO is licensed under the [GNU General Public License Version 3 (GPLv3)](http://www.gnu.org/licenses/gpl.html).

If you want to use this software commercially, please contact us.
## Building the framework
So far, the framework has only been built and tested on the following system.
### Requirements
* [Ubuntu 16.04](https://www.ubuntu.com/)
* [OpenCV > 3](http://opencv.org/)
* [Eigen > 3.3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [Sophus](https://github.com/strasdat/Sophus)

Building on Windows and backwards compatibility might be added in the future.

### Optional
Set the optional packages in the cmake-gui
* [Pangolin](https://github.com/stevenlovegrove/Pangolin)  (for graphical viewer)
* Intel RealSense ZR300 (see below)
* Orbbec Astra Pro (see below)

### Build commands
```bash
git clone REVO
cd REVO
mkdir build
cd build
cmake . ..
make -j
```

## How to reproduce the results from the paper
We provide associate files for the [TUM dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset).
Download the dataset you want to test and specify it in the dataset_tumX.yaml settings file.

In the "REVO" directory:
```bash
build/REVO config/revo_settings.yaml config/dataset_tum1.yaml
```
## Supported Sensors
For Intel set "WITH_REALSENSE" and for ORBBEC set "WITH_ORBBEC_ASTRA_PRO" and make sure that you set the USB rules in a way that the sensor is accessible for every user (default is root only).

### Intel RealSense ZR300
Install [librealsense](https://github.com/IntelRealSense/librealsense), set the intrinsic parameters in the config file.
This framework was tested with the Intel RealSense ZR300.

### Orbbec Astra Pro Sensor
The standard OpenNI driver can only access the depth stream of the [Orbbec Astra Pro Sensor](https://orbbec3d.com/product-astra-pro/), thus we have to access the color stream like a common webcam.
*Note: We use libuvc because the standard webcam interface of [OpenCV](http://opencv.org/) buffers the images and doesn't always return the newest image.*

First [download the openni driver](https://orbbec3d.com/develop/#registergestoos) and choose the correct *.zip file that matches your architecture, e.g. OpenNI-Linux_x64-2.3.zip. 
Extract it and copy libOpenNI2.so and the "Include" and "OpenNI2" folder to REVO_FOLDER/orbbec_astra_pro/drivers. 

Then install [Olaf Kaehler's fork of libuvc](https://github.com/olafkaehler/libuvc) by performing the following steps in the main directory.
```bash
cd ThirdParty
git clone https://github.com/olafkaehler/libuvc
cd libuvc
mkdir build
cd build
cmake . ..
make -j
make install
```



