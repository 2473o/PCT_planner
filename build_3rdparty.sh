#!/bin/bash

ROOT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}"); pwd)

# lvr2_VERSION=main
cJSON_VERSION=1.7.19
gtsam_VERSION=4.2
# gtsam_VERSION=4.1.1
osqp_VERSION=1.0.0
pcl_VERSION=1.15.1

export cJSON_DIR=$ROOT_DIR/3rdparty/cJSON-1.7.19/lib/cmake/cJSON
export GTSAM_DIR=$ROOT_DIR/3rdparty/gtsam-$gtsam_VERSION/lib/cmake
export OSQP_DIR=$ROOT_DIR/3rdparty/osqp-1.0.0/lib/cmake/osqp

export PCL_DIR=$ROOT_DIR/3rdparty/pcl-1.15.1
export PCL_DIR=$PCL_ROOT/share/pcl-1.15
export PCL_INCLUDE_DIRS=$PCL_ROOT/include/pcl-1.15


# build_lvr2(){

# if [ ! -d $ROOT_DIR/src/lvr2-$lvr2_VERSION ]; then
#     git clone -b $lvr2_VERSION --single-branch https://github.com/uos/lvr2.git  $ROOT_DIR/src/lvr2-$lvr2_VERSION
# else
#     echo "lvr2 exists"
#     git switch $lvr2_VERSION
# fi

# if [ ! -d $ROOT_DIR/src/lvr2-$lvr2_VERSION/build ]; then
#     mkdir $ROOT_DIR/src/lvr2-$lvr2_VERSION/build
# else
#     rm -rf $ROOT_DIR/src/lvr2-$lvr2_VERSION/build/*
# fi

# cd $ROOT_DIR/src/lvr2-$lvr2_VERSION/build

# cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release \
#  -DCMAKE_INSTALL_PREFIX=$ROOT_DIR/lvr2-$lvr2_VERSION \
#  -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
#  -DCMAKE_C_COMPILER_LAUNCHER=ccache

# ninja && ninja install
# }

build_package(){
package_name=$1
package_version=$2
shift 2
cmake_args=("$@")

if [ ! -d $ROOT_DIR/src/$package_name-$package_version/build ]; then
    mkdir $ROOT_DIR/src/$package_name-$package_version/build
else
    rm -rf $ROOT_DIR/src/$package_name-$package_version/build/*
fi

cd $ROOT_DIR/src/$package_name-$package_version/build

cmake .. -GNinja \
 -DCMAKE_BUILD_TYPE=Release \
 -DCMAKE_INSTALL_PREFIX=$ROOT_DIR/$package_name-$package_version \
 -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
 -DCMAKE_C_COMPILER_LAUNCHER=ccache \
  "${cmake_args[@]}"

 ninja && ninja install
}

build_cjson(){
package_name="cJSON"

build_package $package_name $cJSON_VERSION
}

build_gtsam(){
package_name="gtsam"

build_package $package_name $gtsam_VERSION \
    -DGTSAM_USE_SYSTEM_EIGEN=ON
}

build_osqp(){
package_name="osqp"

build_package $package_name $osqp_VERSION
}

build_pcl(){
package_name="pcl"

build_package $package_name $pcl_VERSION \
 -DCMAKE_PREFIX_PATH="$ROOT_DIR/cJSON-$cJSON_VERSION"
}

#  "lvr2")
#    build_lvr2
#    ;;
case $1 in
"cjson")
    build_cjson
    ;;
"gtsam")
    build_gtsam
    ;;
    "osqp")
    build_osqp
    ;;
"pcl")
    build_pcl
    ;;
"all")
    build_cjson
    build_gtsam
    build_osqp
    build_pcl
    ;;
*)
    echo "Usage:  {cjson|gtsam|osqp|pcl|all}"
    exit 1
esac