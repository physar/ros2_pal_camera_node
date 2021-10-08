#!bin/bash

pal_sdk_dir=$1

cd ./include/dreamvu
ln -s $pal_sdk_dir/include/PAL.h .
ln -s $pal_sdk_dir/include/PAL_Image.h .
ln -s $pal_sdk_dir/include/PAL_CameraProperties.h .
cd ../..

cd ./lib/dreamvu
ln -s $pal_sdk_dir/lib/libPAL.so .
ln -s $pal_sdk_dir/lib/libPAL_CAMERA.so .
ln -s $pal_sdk_dir/lib/libPAL_DEPTH.so .
ln -s $pal_sdk_dir/lib/libPAL_DE.so 
ln -s $pal_sdk_dir/lib/libPAL_SDD.so 

