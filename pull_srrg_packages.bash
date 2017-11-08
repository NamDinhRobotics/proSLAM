#!/bin/bash
#ds script is assumed to be called from within proslam project root (brutal)
DIR_ORIGINAL=`pwd`
#ds move one level up (must be src now, brutal)
cd ..
DIR_CATKIN_SOURCE=`pwd`
echo "- updating SRRG packages in: ${DIR_CATKIN_SOURCE} (this should be your catkin workspace source directory)"
cd ${DIR_CATKIN_SOURCE}

#ds start loading packages - first check if they exist
if [ -d "${DIR_CATKIN_SOURCE}/srrg_cmake_modules" ]; then
    echo "- srrg_cmake_modules already installed - updating code"
    cd "${DIR_CATKIN_SOURCE}/srrg_cmake_modules"
    git pull
else
    cd "${DIR_CATKIN_SOURCE}"
    echo "- installing srrg_cmake_modules:"
    git clone https://gitlab.com/srrg-software/srrg_cmake_modules
fi
if [ -d "${DIR_CATKIN_SOURCE}/srrg_core" ]; then
    echo "- srrg_core already installed - updating code"
    cd "${DIR_CATKIN_SOURCE}/srrg_core"
    git pull
else
    cd "${DIR_CATKIN_SOURCE}"
    echo "- installing srrg_core:"
    git clone https://gitlab.com/srrg-software/srrg_core
fi
if [ -d "${DIR_CATKIN_SOURCE}/srrg_gl_helpers" ]; then
    echo "- srrg_gl_helpers already installed - updating code"
    cd "${DIR_CATKIN_SOURCE}/srrg_gl_helpers"
    git pull
else
    cd "${DIR_CATKIN_SOURCE}"
    echo "- installing srrg_gl_helpers:"
    git clone https://gitlab.com/srrg-software/srrg_gl_helpers
fi
if [ -d "${DIR_CATKIN_SOURCE}/srrg_core_viewers" ]; then
    echo "- srrg_core_viewers already installed - updating code"
    cd "${DIR_CATKIN_SOURCE}/srrg_core_viewers"
    git pull
else
    cd "${DIR_CATKIN_SOURCE}"
    echo "- installing srrg_core_viewers:"
    git clone https://gitlab.com/srrg-software/srrg_core_viewers
fi
if [ -d "${DIR_CATKIN_SOURCE}/srrg_hbst" ]; then
    echo "- srrg_hbst already installed - updating code"
    cd "${DIR_CATKIN_SOURCE}/srrg_hbst"
    git pull
else
    cd "${DIR_CATKIN_SOURCE}"
    echo "- installing srrg_hbst:"
    git clone https://gitlab.com/srrg-software/srrg_hbst
fi

#ds notify and return to original folder
echo "- done"
cd ${DIR_ORIGINAL}
