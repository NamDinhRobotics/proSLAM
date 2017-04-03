| **ProSLAM** |
| :------: |
| Dominik Schlegel, Mirco Colosi, Giorgio Grisetti |
As this is a working repository, none of the code is assumed to be static.
For related publications please refer to revision 8bce6d75

### Demo videos ###
[ProSLAM: Full run KITTI Sequence 00 updated (real-time, 1 thread@2.40GHz/i7-4700MQ)][kitti_00_updated]

[ProSLAM: Full run KITTI Sequence 00 (real-time, 1 thread@2.40GHz/i7-4700MQ)][kitti_00]

[ProSLAM: Full run KITTI Sequence 01 (real-time, 1 thread@2.40GHz/i7-4700MQ)][kitti_01]

[ProSLAM: Full run KITTI Sequence 06 (real-time, 1 thread@3.50GHz/i7-4770K)][kitti_06]

[ProSLAM: Full run KITTI Sequence 10 (real-time, 1 thread@2.40GHz/i7-4700MQ)][kitti_10]

[ProSLAM: Full run EuRoC MH_01_easy (real-time, 1 thread@3.50GHz/i7-4770K)][euroc_01]

(All of the above are clickable YouTube links)

[kitti_00_updated]: https://www.youtube.com/watch?v=hIeaB-MMJMo
[kitti_00]: https://www.youtube.com/watch?v=n_UmEpIwb9Y
[kitti_01]: https://www.youtube.com/watch?v=iGSCOEn5Nx8
[kitti_06]: https://www.youtube.com/watch?v=Bmig0ASFOY4
[kitti_10]: https://www.youtube.com/watch?v=ZW8OQ2b0tjk
[euroc_01]: https://www.youtube.com/watch?v=TctS1b1zCbY

---
### Supported environments ###
Currently Linux only:
 - Ubuntu 14.04 LTS + ROS Indigo /(OpenCV2 + Qt4)
 - Ubuntu 16.04 LTS + ROS Kinetic/(OpenCV3 + Qt5)<br/>

The complete system **runs on a single thread** (visualization components are synchronous)

---
### Code statistics | Revision 8bce6d75 ###

    cloc srrg_proslam/src/

| http://cloc.sourceforge.net v 1.60 |
| :-: |


| Language     | files  | blank lines | comment lines | code lines |
| :----------- | :----- | :---------- | :------------ | :--------- |
| C++          | 18     | 739         | 637           | 2891       |
| C/C++ Header | 18     | 409         | 289           | 1115       |
| CMake        | 8      | 9           | 2             | 86         |
| SUM:         | **44** | **1157**    | **928**       | **4092**   |

---
### How do I get set up? ###
1) install the Ubuntu packages (`ninja-build` is not required when using an existing g2o installation)

    sudo apt-get install build-essential libeigen3-dev libsuitesparse-dev freeglut3-dev libqglviewer-dev ninja-build

---
2) download and install
 - ROS: http://wiki.ros.org/ROS/Installation

or (OpenCV + Qt)
 - OpenCV3: https://github.com/opencv/opencv/archive/3.2.0.zip (Version 3.2.0) - used for FAST detection, BRIEF extraction, visualization
 - Qt5: https://wiki.qt.io/Install_Qt_5_on_Ubuntu (Version 5.7.0)              - used for visualization

---
3) download and install the colorful `Catkin Command Line Tools`: https://catkin-tools.readthedocs.io/en/latest/installing.html (alternatively one can also use `ROS catkin`):

    sudo apt-get install python-catkin-tools

---
4) set the environment variable `$G2O_ROOT` to use your own g2o installation - or clone `g2o for catkin` (https://github.com/yorsh87/g2o_catkin) to your catkin workspace:

    git clone https://github.com/yorsh87/g2o_catkin.git
    
and build it (slow as it will perform a download using unladen swallows):
    
    catkin build g2o_catkin

---
5) download this repository to your catkin workspace:

    git clone https://gitlab.com/srrg-software/srrg_proslam.git
    
enter the project directory in your catkin workspace (e.g. `../src/srrg_proslam`) and fetch the modular SRRG libraries by executing the script:

    ./pull_srrg_packages.bash
    
then build the project using:
    
    catkin build srrg_proslam

CMake variables that must be set when building without ROS or to select specific libraries:

    -D OpenCV_DIR=/your/path/to/the/opencv/build/folder
    -D G2O_ROOT=/your/path/to/the/g2o/root/folder

---
### How do I check if it works? ###

1) download the `KITTI Sequence 00` into a folder on your computer: https://drive.google.com/open?id=0ByaBRAPfmgEqdXhJRmktQ2lsMEE (2.8GB)

---
2) launch a terminal in that folder and uncompress the tarball:

    tar -xzvf 00.tar.gz

The folder should now contain 4 files (.txt) and 1 folder (.txt.d) plus the tarball 00.tar.gz

---
3) run the system directly in the folder (`rosrun` is used for convenience only, the binary can also be launched normally with `./srrg_proslam_app`):

    rosrun srrg_proslam srrg_proslam_app 00.txt -use-gui -show-top

Three windows will pop up - "input: images", "output: map (bird view)" and "output: map (top view)"

---
4) press `[Backspace]` on the input window to toggle between automatic processing and stepwise (press `[Space]` for stepping) mode

---
5) press `[H]` to view the available commands for the output windows (Number keys `1`-`8`)

---
6) press `[Esc]` to terminate the system prematurely` (in an input or output window)

---
7) to see the raw system performance simply launch srrg_proslam without any parameters other than the input dataset:

    rosrun srrg_proslam srrg_proslam_app 00.txt

---
### Pre-formatted datasets ###

 - `KITTI Sequence 00`: https://drive.google.com/open?id=0ByaBRAPfmgEqdXhJRmktQ2lsMEE (2.8GB)
 - `KITTI Sequence 01`: https://drive.google.com/open?id=0ByaBRAPfmgEqN19hTUJjRG9XV3M (0.7GB)
 - `KITTI Sequence 04`: https://drive.google.com/open?id=0ByaBRAPfmgEqOEhEdEdLcUJUMlE (0.2GB)
 - `EuRoC MH_01_easy`: https://drive.google.com/open?id=0ByaBRAPfmgEqbUctejZwb0xRaFk (1.9GB)<br/>

Run procedure remains identical to the one above (only the dataset name has to be adjusted, e.g. `00.txt` becomes `MH_01_easy.txt`)

---
### Custom stereo camera sensor input / ROS node ###

On-the-fly raw stereo image processing with custom stereo camera parameters will be supported shortly.<br/>
Please use the provided datasets in SRRG format.<br/>

The ROS node (`srrg_proslam_node`) is currently under development.

---
### It doesn't work? ###

Feel free to contact the maintainer at any time (see package.xml)
