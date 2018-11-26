    
    ProSLAM: Programmers SLAM

Contributors: Dominik Schlegel, Mirco Colosi, Giorgio Grisetti <br/>

**ProSLAM uses the lightning-fast, header-only [HBST library](https://gitlab.com/srrg-software/srrg_hbst) for binary descriptor similarity search (loop closing)**

---
### Demo videos ###
Current state: <br/>
[ProSLAM: KITTI Sequence 00 at 150 Hz](https://www.youtube.com/watch?v=Lio_M_O1VhI) <br/>

Previous versions: <br/>
[ProSLAM: Full run KITTI Sequence 00 updated (real-time, 1 thread@2.40GHz/i7-4700MQ)](https://www.youtube.com/watch?v=hIeaB-MMJMo) <br/>
[ProSLAM: Full run KITTI Sequence 00 (real-time, 1 thread@2.40GHz/i7-4700MQ)](https://www.youtube.com/watch?v=n_UmEpIwb9Y) <br/>
[ProSLAM: Full run KITTI Sequence 01 (real-time, 1 thread@2.40GHz/i7-4700MQ)](https://www.youtube.com/watch?v=iGSCOEn5Nx8) <br/>
[ProSLAM: Full run KITTI Sequence 06 (real-time, 1 thread@3.50GHz/i7-4770K)](https://www.youtube.com/watch?v=Bmig0ASFOY4) <br/>
[ProSLAM: Full run KITTI Sequence 10 (real-time, 1 thread@2.40GHz/i7-4700MQ)](https://www.youtube.com/watch?v=ZW8OQ2b0tjk) <br/>
[ProSLAM: Full run EuRoC MH_01_easy (real-time, 1 thread@3.50GHz/i7-4770K)](https://www.youtube.com/watch?v=TctS1b1zCbY) <br/>
(All of the above are clickable YouTube links)

---
### Supported environments ###
 - Ubuntu 14.04 LTS (**gcc 5**) + ROS Indigo / OpenCV2 + Qt4 + g2o (see Note for old version)
 - Ubuntu 16.04 LTS (gcc 5) + ROS Kinetic / OpenCV3 + Qt5 + g2o (current)
 - Ubuntu 18.04 LTS (gcc 7) + OpenCV3 + Qt5 + g2o (current)

The complete SLAM system **runs on a single thread** (a second thread is launched for optional visualization) <br/>
ProSLAM features an extensive [parameter configuration][proslam_wiki] on all SLAM layers and 4 different logging levels

---
### How do I get set up? ###
1) install the Ubuntu packages

        sudo apt-get install build-essential libeigen3-dev libsuitesparse-dev freeglut3-dev libqglviewer-dev libyaml-cpp-dev

2) download and install
    - ROS: http://wiki.ros.org/ROS/Installation

    or (OpenCV + Qt)
    - OpenCV3: https://github.com/opencv/opencv/archive/3.2.0.zip (Version 3.2.0) - used for FAST detection, BRIEF extraction, visualization
    - Qt5: https://wiki.qt.io/Install_Qt_5_on_Ubuntu (Version 5.7.0)              - used for visualization

3) download and install the colorful `Catkin Command Line Tools`: https://catkin-tools.readthedocs.io/en/latest/installing.html (alternatively one can also use `ROS catkin`):

        sudo apt-get install python-catkin-tools

4) set the environment variable `$G2O_ROOT` to use your own g2o installation - or clone `g2o for catkin` (https://github.com/yorsh87/g2o_catkin) to your catkin workspace:

        sudo apt-get install ninja-build
        git clone https://github.com/yorsh87/g2o_catkin.git
    
    and build it (slow as it will perform a download using unladen swallows):
    
        catkin build g2o_catkin

    Note: If one is using a g2o version with the old ownership model ([3a740d8](https://github.com/RainerKuemmerle/g2o/commit/3a740d8922e524bf832c66aaf5c8f59c70aec604) or earlier) the line:

    `add_definitions(-DSRRG_PROSLAM_G2O_HAS_NEW_OWNERSHIP_MODEL)`

    in the root `CMakeLists.txt` must be commented for proper compilation.

    Note: Make sure to copy or link the generated `config.h` header file of your g2o build into your g2o include folder (in case of a custom build).

5) download this repository to your catkin workspace:

        git clone https://gitlab.com/srrg-software/srrg_proslam.git
    
6) enter the project directory in your catkin workspace (e.g. `../src/srrg_proslam`) and fetch the modular SRRG libraries by executing the script:

        ./pull_srrg_packages.bash

7) then build the project using (`ROS catkin` requires: `catkin_make -pkg` instead of `catkin build`):
    
        catkin build srrg_proslam

CMake variables that must be set when building without ROS or to select specific libraries:

    -D OpenCV_DIR=/your/path/to/the/opencv/build/folder
    -D G2O_ROOT=/your/path/to/the/g2o/root/folder

---
### How do I check if it works? ###
1) download the [KITTI Sequence 00](https://drive.google.com/open?id=0ByaBRAPfmgEqdXhJRmktQ2lsMEE) to your computer.

2) launch a terminal in that folder and uncompress the tarball:

        tar -xzvf 00.tar.gz

    The folder should now contain 4 files (.txt) and 1 folder (.txt.d) plus the tarball 00.tar.gz

3) run the system directly in the folder (`rosrun` is used for convenience only, the binary can also be launched normally with `./app`):

        rosrun srrg_proslam app 00.txt -use-gui

    Two windows will pop up - "input: images" (OpenCV), "output: map" (OpenGL) <br/>
    **all controls are handled in the "output: map" window**

4) press `[Space]` on the "output: map" window to toggle between automatic processing and stepwise (press `[ARROW_UP]` for stepping) mode

5) press `[H]` to view the available commands for the "output: map" viewer (Number keys `1`-`8`)

6) press `[Esc]` to terminate the system prematurely

---
### Quantitative result evaluation ###
To see the raw system performance on KITTI simply launch srrg_proslam with the input dataset and the configuration file:

        rosrun srrg_proslam app 00.txt -c configuration_kitti.yaml

After a complete run we evaluate the `KITTI error statistics` by calling:

        rosrun srrg_core srrg_kitti_evaluate_odometry_app -odom trajectory_kitti.txt -gt 00_gt.txt -seq 00.txt
    
To see the raw system performance on EuRoC simply launch srrg_proslam with the input dataset and the configuration file:

        rosrun srrg_proslam app MH_01_easy.txt -c configuration_euroc.yaml

After a complete run we evaluate the `EuRoC RMSE` by calling:

        rosrun srrg_proslam trajectory_analyzer -tum trajectory_tum.txt -asl state_groundtruth_estimate.csv
    
The specific configuration files (`configuration_kitti.yaml, configuration_euroc.yaml`) can be found in the configurations folder of the ProSLAM project

---
### Pre-formatted SRRG datasets (online ground truth display) ###
 - `KITTI Sequence 00`: https://drive.google.com/open?id=0ByaBRAPfmgEqdXhJRmktQ2lsMEE (2.8GB)
 - `KITTI Sequence 01`: https://drive.google.com/open?id=0ByaBRAPfmgEqN19hTUJjRG9XV3M (0.7GB)
 - `KITTI Sequence 03`: https://drive.google.com/open?id=1ySsnZoMT_HotLWF9ToyWIa3Ud_YhSEKj (0.6GB)
 - `KITTI Sequence 04`: https://drive.google.com/open?id=0ByaBRAPfmgEqOEhEdEdLcUJUMlE (0.2GB)
 - `KITTI Sequence 06`: https://drive.google.com/open?id=0ByaBRAPfmgEqcC14TS1mbF9XSmc (0.7GB)
 - `KITTI Sequence 10`: https://drive.google.com/open?id=1Nt71O0i0ClZwa6W8PrybjlNV9Xq99S1y (0.7GB)
 - `EuRoC MH_01_easy`: https://drive.google.com/open?id=0ByaBRAPfmgEqbUctejZwb0xRaFk (1.3GB)
 - `EuRoC MH_05_difficult`: https://drive.google.com/open?id=0ByaBRAPfmgEqTWVCZDVqNTY2QXc (0.8GB)
 - `EuRoC V1_01_easy`: https://drive.google.com/open?id=0ByaBRAPfmgEqRW5aWUZWV1NLSVE (1.0GB) <br/>

Run procedure remains identical to the one above (only the dataset name has to be adjusted, e.g. `00.txt` becomes `MH_01_easy.txt`) <br/>
The ground truth display can be toggled by pressing the Number key `4` in the "output: map" window <br/>
The EuRoC sequences generally require image histogram equalization for best performance (option `-equalize-histogram/-eh`)

Dataset conversion utilities are available in the [srrg_core](https://gitlab.com/srrg-software/srrg_core) package <br/>
An example to obtain a converted EuRoC sequence (e.g. `MH_01_easy.txt`, using a hardcoded camera calibration) goes as follows:

    rosrun srrg_core srrg_message_converter_euroc_app -o MH_01_easy.txt -hard-calibration

Note that the command has to be issued from inside of the ASL folder `mav0` <br/>

---
### Custom stereo camera sensor input / ROS node ###
On-the-fly raw stereo image processing with custom stereo camera parameters will be supported shortly. <br/>
Please use the provided datasets in SRRG format. <br/>

The ROS node is currently under development.

---
### [Configuration file][proslam_wiki] (YAML) ###
ProSLAM supports classic YAML configuration files, enabling fine-grained adjustment of deep system parameters. <br/>
Example configuration files can be found in the `configurations` folder. <br/>
A custom configuration file can be specified as follows:

    rosrun srrg_proslam app 00.txt -c configuration.yaml
    
[proslam_wiki]: https://gitlab.com/srrg-software/srrg_proslam/wikis/home

---
### Full system overview (might take some time to load)
![](https://drive.google.com/uc?export=download&id=1U8SJuETvbtYgqGFVIPffUPwI-iikSs4s)
Detailed notes coming soon!

---
### It doesn't work? ###
[Open an issue](https://gitlab.com/srrg-software/srrg_proslam/issues) or contact the maintainer (see package.xml)

 - 3D viewer issues (Qt) on Ubuntu 18.04? Set the enviroment variable: `QT_QPA_PLATFORMTHEME="gtk"` and try again

---
## Related publications
Please cite our most recent article when using the ProSLAM system: <br>

    @inproceedings{2018-schlegel-proslam, 
      author    = {D. Schlegel and M. Colosi and G. Grisetti}, 
      booktitle = {2018 IEEE International Conference on Robotics and Automation (ICRA)}, 
      title     = {{ProSLAM: Graph SLAM from a Programmer's Perspective}}, 
      year      = {2018},
      pages     = {1-9}
    }

> ICRA 2018 'ProSLAM: Graph SLAM from a Programmer's Perspective' https://ieeexplore.ieee.org/document/8461180/ (DOI: 10.1109/ICRA.2018.8461180)
