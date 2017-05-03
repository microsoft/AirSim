# Install Eigen

Eigen is installed by the [build process](build.md) into your AirSim folder, so after the build completes
you will have ~/AirSim/eigen.  The build is downloading eigen from [Eigen 3.3.2 zip file](http://bitbucket.org/eigen/eigen/get/3.3.2.zip) 

But your Unreal Projects will also need to be able to find this location and for that we set an EIGEN_ROOT environment variable.
That variable needs to point to the eigen folder (not the eigen3 subfolder)

## Windows

See how to [set environment variables](http://www.computerhope.com/issues/ch000549.htm) on windows.

## Linux 

On Linux you can use the usual trick if your current folder is set to your AirSim git repo:

````
echo export EIGEN_ROOT=$(pwd)/eigen >> ~/.bashrc
source  ~/.bashrc
````

