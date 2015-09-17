Fast Plane Extraction Using Agglomerative Hierarchical Clustering (AHC)
=======================================================================

Legal Remarks
-------------

Copyright 2014 Mitsubishi Electric Research Laboratories All
Rights Reserved.

Permission to use, copy and modify this software and its
documentation without fee for educational, research and non-profit
purposes, is hereby granted, provided that the above copyright
notice, this paragraph, and the following three paragraphs appear
in all copies.

To request permission to incorporate this software into commercial
products contact: Director; Mitsubishi Electric Research
Laboratories (MERL); 201 Broadway; Cambridge, MA 02139.

IN NO EVENT SHALL MERL BE LIABLE TO ANY PARTY FOR DIRECT,
INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
DOCUMENTATION, EVEN IF MERL HAS BEEN ADVISED OF THE POSSIBILITY OF
SUCH DAMAGES.

MERL SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
"AS IS" BASIS, AND MERL HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE,
SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

Overview
--------

This source code package contains our C++ implementation of the AHC based fast plane extraction for organized point cloud (point cloud that can be indexed as an image). There are three folders in this package:

* include

    Our C++ implementation of the algorithm with dependencies on OpenCV and shared_ptr (from C++11 or Boost).

* cpp

    Two example C++ console applications using our algorithm to extract planes from Kinect-like point cloud (depends on PCL), with a CMake script to help generating project files.

* matlab

    A matlab interface (fitAHCPlane.m) through MEX for using our algorithm in matlab. We also provide a wrapper class Kinect.m and kinect_ahc.m to do real-time plane extraction in matlab, partially depends on a 3rd-party toolbox [Kinect_Matlab].
    
**If you use this package, please cite our ICRA 2014 paper:**

Feng, C., Taguchi, Y., and Kamat, V. R. (2014). "Fast Plane Extraction in Organized Point Clouds Using Agglomerative Hierarchical Clustering." Proceedings of the IEEE International Conference on Robotics and Automation, Hong Kong, China, 6218-6225.
    
[Kinect_Matlab]:http://www.mathworks.com/matlabcentral/fileexchange/30242-kinect-matlab

Version
-------

1.0

Installation
------------

##### C++ example

1. Install OpenCV, Boost and PCL (If you install PCL using their all-in-one installer, you directly get Boost installed as well).

2. Generate project file using CMake under either Windows or Linux.

3. Compile.

4. Run the compiled process: plane_fitter (first connect a Kinect to your computer!) or plane_fitter_pcd (first modify plane_fitter_pcd.ini accordingly!).

5. Enjoy!

##### Matlab example

1. In matlab:
```
cd WHERE_YOU_EXTRACT_THE_PACKAGE/matlab/mex
```

2. Run makefile.m

3. Select the directories for *OpenCV_Include*, *OpenCV_Lib*, and *Boost_Include* respectively

4. If everything compiles smoothly:
```
cd ..
```

5. Load a single frame we've prepared for you in matlab by:
```
load frame
```

6. Run our algorithm on the point cloud:
```
frame.mbs=fitAHCPlane(frame.xyz);
viewSeg(frame.mbs,640,480)
```

7. Enjoy!

8. If you want to play with the kinect_ahc.m with a Kinect, install [Kinect_Matlab] first.

Contact
-------

Chen Feng <simbaforrest at gmail dot com>

**Feel free to email any bugs or suggestions to help us improve the code. Thank you!**