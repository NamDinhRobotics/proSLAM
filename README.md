| **ProSLAM** |
| :------: |
| Dominik Schlegel, Mirco Colosi, Giorgio Grisetti |

### Demo videos ###
![ProSLAM: Full run KITTI Sequence 00 (real-time, 1 thread@2.40GHz/i7-4700MQ)](https://www.youtube.com/watch?v=n_UmEpIwb9Y)
![ProSLAM: Full run KITTI Sequence 06 (real-time, 1 thread@3.50GHz/i7-4770K)](https://www.youtube.com/watch?v=Bmig0ASFOY4)
!(ProSLAM: Full run EuRoC MH_01_easy (real-time, 1 thread@3.50GHz/i7-4770K))(https://www.youtube.com/watch?v=TctS1b1zCbY)
(All of the above are clickable YouTube links)

---
### Supported environments ###
Currently Linux only:
 - Ubuntu 14.04 LTS + ROS Indigo /(OpenCV2 + Qt4)
 - Ubuntu 16.04 LTS + ROS Kinetic/(OpenCV3 + Qt5)<br/>

The complete system **runs on a single thread** (including visualization components)

---
### How do I get set up? ###
1) install the Ubuntu packages

    sudo apt-get install build-essential libeigen3-dev libsuitesparse-dev freeglut3-dev libqglviewer-dev ninja-build

---
2) download and install
 - ROS: http://wiki.ros.org/ROS/Installation

or (OpenCV + Qt)
 - OpenCV3: https://github.com/opencv/opencv/archive/3.2.0.zip (Version 3.2.0) - used for FAST detection, BRIEF extraction, visualization
 - Qt5: https://wiki.qt.io/Install_Qt_5_on_Ubuntu (Version 5.7.0)              - used for visualization

---
3) download and install the Colorful Catkin Command Line Tools: https://catkin-tools.readthedocs.io/en/latest/installing.html (currently required) if ROS is installed simply enter:

    sudo apt-get install python-catkin-tools

---
4) clone g2o for catkin (currently required) to your catkin workspace:

    git clone https://github.com/yorsh87/g2o_catkin.git
    
and build it:
    
    catkin build g2o_catkin

---
5) download this repository to your catkin workspace:

    git clone https://gitlab.com/srrg-software/srrg_proslam.git
    
enter the project directory in your catkin workspacce (e.g. ../src/srrg_proslam) and fetch the modular SRRG libraries by executing the script:

    ./pull_srrg_packages.bash
    
then build the project using:
    
    catkin build srrg_proslam

CMake variables that must be set when building without ROS or to select specific libraries:

    -D OpenCV_DIR=/your/path/to/the/opencv/build/folder

---
### How do I check if it works? ###

1) download the KITTI sequence 00 into a folder on your computer: https://drive.google.com/open?id=0ByaBRAPfmgEqdXhJRmktQ2lsMEE (2.8GB)

---
2) launch a terminal in that folder and uncompress the tarball:

    tar -xzvf 00.tar.gz

The folder should now contain 4 files (.txt) and 1 folder (.txt.d) plus the tarball 00.tar.gz

---
3) run the system directly in the folder:

    rosrun srrg_proslam srrg_proslam_app 00.txt -use-gui

Three windows will pop up - "input: images", "output: map (bird view)" and "output: map (top view)"

---
4) press [Backspace] on the input window to toggle between automatic processing and stepwise (press [Space] for stepping) mode

---
5) press [H] to view the available commands for the output windows (Number keys 1-8)

---
6) press [Esc] to terminate the system prematurely

---
### Additional pre-formatted datasets ###

EuRoC MH_01_easy: https://drive.google.com/open?id=0ByaBRAPfmgEqbUctejZwb0xRaFk (1.9GB)

Run procedure remains identical to the one above (only the dataset name has to be adjusted, e.g. 00.txt -> MH_01_easy.txt)

---
### Custom input ###

On-the-fly raw stereo image processing with custom stereo camera parameters will be supported shortly.

---
### It doesn't work? ###

Feel free to contact the maintainer at any time (see package.xml)
