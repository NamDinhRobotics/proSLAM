    
    ProSLAM: Programmers SLAM

Contributors: Dominik Schlegel, Mirco Colosi, Giorgio Grisetti <br/>

**ProSLAM uses the lightning-fast, header-only [HBST library](https://gitlab.com/srrg-software/srrg_hbst) for binary descriptor similarity search (loop closing)**

---
### SLAM library applications ###
**app: main application for synchronous image processing (SRRG, KITTI, TUM, ASL, MIT)**

    ./app 00.txt

**node: main application for asynchronous image processing (ROS)**

	rosrun srrg_proslam node -c configuration.yaml

---
### Utilities ###

**stereo_calibrator: utility for calibrating a stereo camera with an SRRG or ASL checkerboard calibration sequence (e.g. EuRoC)**

	./stereo_calibrator -asl cam0 cam1 -o calibration.txt

**test_stereo_frontend: utility for testing the feature-based stereo matching, triangulation and tracking (atm KITTI only)**

	./test_stereo_frontend image_0/000000.png image_1/000000.png calib.txt 50 gt.txt

**trajectory_analyzer: utility for loading and aligning a pair of trajectories (TUM/ASL format)**

	./trajectory_analyzer -tum query_trajectory.txt -asl reference_trajectory.txt

**trajectory_converter: utility for converting g2o pose graphs or TUM/ASL trajectories to KITTI format**

	./trajectory_converter -g2o pose_graph.g2o

---
### It doesn't work? ###
[Open an issue](https://gitlab.com/srrg-software/srrg_proslam/issues) or contact the maintainer (see package.xml)

Instant troubleshooting:
 - 3D viewer issues (Qt) on Ubuntu 18.04? Set the enviroment variable: `QT_QPA_PLATFORMTHEME="gtk"` and try again
