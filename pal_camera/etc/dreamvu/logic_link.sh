#!/bin/bash

pal_sdk_dir=$1

cd ./include/dreamvu
ln -sf $pal_sdk_dir/include/PAL.h .
ln -sf $pal_sdk_dir/include/PAL_Image.h .
ln -sf $pal_sdk_dir/include/PAL_CameraProperties.h .
cd ../..

cd ./lib/dreamvu
ln -sf $pal_sdk_dir/lib/libPAL.so .
ln -sf $pal_sdk_dir/lib/libPAL_CAMERA.so .
ln -sf $pal_sdk_dir/lib/libPAL_DEPTH.so .
ln -sf $pal_sdk_dir/lib/libPAL_DE.so 
ln -sf $pal_sdk_dir/lib/libPAL_SSD.so 
cd ../..

mkdir -p ~/local/etc/dreamvu
cp -p ./etc/dreamvu/SavedPalProperties.txt ~/.local/etc/dreamvu/
