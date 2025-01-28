# Log Band Power
Processing chain in ROS-Neuro to determine when the logarithmic bandpower of the EEG signal crosses a given threshold.

## Dataset
The GDF file provided was recorded using a 16-channel EEG amplifier (g.USBamp, g.Tec) at a sampling rate of 512 Hz, where the electrodes were positioned according to the 10-20 international system.

### Resource
https://drive.google.com/file/d/1uBr5xO4rIT2c4uyMv3Wp68hWRpg_plb_/view?usp=sharing

## Instructions

1. Source ROS workspace
```bash
source /opt/ros/noetic/setup.bash
```

2. Initialize a catkin workspace called `catkin_ws`
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src/
catkin_init_workspace
```

3. Compile the catkin workspace from directory `catkin_ws/` and source it
```bash
catkin build
# or
catkin_make

source ./devel/setup.bash
```

4. Copy `logbandpower` package in directory `catkin_ws/src/`

5. If it does not exist, create directory `record` in package directory `catkin_ws/src/logbandpower/`

6. Create directory `dataset` in package directory `catkin_ws/src/logbandpower/`

7. Rename the input GDF file as `data.gdf` and copy it in directory `catkin_ws/src/logbandpower/dataset/`

8. Compile the catkin workspace from directory `catkin_ws/` and source it
```bash
catkin build
# or
catkin_make

source ./devel/setup.bash
```

9. Run launch file `logbandpower.launch` to start the analysis from directory `catkin_ws/`
```bash
source ./devel/setup.bash
roslaunch logbandpower logbandpower.launch
```

10. Run launch file `visualization.launch` to visualize the signals from directory `catkin_ws/`
```bash
source ./devel/setup.bash
roslaunch logbandpower visualization.launch
```
 
11. Once the analysis is completed, the ROS bag `result.bag` is saved in directory `catkin_ws/src/logbandpower/record/` 

### Tested Environments
- Local machine, Ubuntu 20.04.6 LTS, Python 3.8.10, ROS Noetic, catkin workspace build with command `catkin build`
- Local machine, Ubuntu 20.04.6 LTS, Python 3.8.10, ROS Noetic, catkin workspace build with command `catkin_make`

#### VLAB Environment
Due to an older version of ROS Neuro installed in the VLAB virtual machine, a modification is required to make the package running correctly.

It is required to:
1. Move the input GDF file `data.gdf` in directory `home/user/`
2. Comment `Option 1` (default) and decomment `Option 2` in the launch file `acquisition.launch`
```xml
<!-- Option 2: VLAB virtual machine -->
<arg name="devarg" default="/home/user/data.gdf"/>
```

## Authors
- Federico Pivotto, 2121720, federico.pivotto@studenti.unipd.it
- Alessandro Bozzon, 2122185, alessandro.bozzon@studenti.unipd.it
- Riccardo Simion, 2157564, riccardo.simion@studenti.unipd.it
- Riccardo Zerbinati, 2158676, riccardo.zerbinati@studenti.unipd.it

### Contributions
| Member             | Workload | Work                                                                |
| ------------------ | :------: | ------------------------------------------------------------------- |
| Federico Pivotto   | 25%      | Setup `CMakeLists.txt`, config `package.xml`, script `bandpower.py` |
| Alessandro Bozzon  | 25%      | Config `ChainCfg.yaml`, script `thresholding.py`                    |
| Riccardo Simion    | 25%      | Script `record.py`, code debugging                                  |
| Riccardo Zerbinati | 25%      | Component launch files, launch file `logbandpower.launch`           |
