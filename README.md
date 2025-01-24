# Log Band Power
Processing chain in ROS-Neuro to determine when the logarithmic bandpower of the EEG signal crosses a given threshold.

## Dataset
The GDF file provided was recorded using a 16-channel EEG amplifier (g.USBamp, g.Tec) at a sampling rate of 512 Hz, where the electrodes were positioned according to the 10-20 international system.

### Resource
https://drive.google.com/file/d/1uBr5xO4rIT2c4uyMv3Wp68hWRpg_plb_/view?usp=sharing

# Instructions
1. Rename the input GDF file as `data.gdf`

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