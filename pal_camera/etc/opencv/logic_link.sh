#!/bin/bash

cd ./lib/opencv 
ln -s libopencv_calib3d.so.3.4.4 libopencv_calib3d.so.3.4
ln -s libopencv_core.so.3.4.4 libopencv_core.so.3.4
ln -s libopencv_features2d.so.3.4.4 libopencv_features2d.so.3.4
ln -s libopencv_flann.so.3.4.4 libopencv_flann.so.3.4
ln -s libopencv_imgcodecs.so.3.4.4 libopencv_imgcodecs.so.3.4
ln -s libopencv_imgproc.so.3.4.4 libopencv_imgproc.so.3.4
ln -s libopencv_videoio.so.3.4.4 libopencv_videoio.so.3.4
ln -s libopencv_ximgproc.so.3.4.4 libopencv_ximgproc.so.3.4
cd ../..
