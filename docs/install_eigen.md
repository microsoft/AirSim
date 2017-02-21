# Install Eigen

1. Download Eigen (at least v3.3.x): [Eigen 3.3.2 zip file](http://bitbucket.org/eigen/eigen/get/3.3.2.zip).
2. Create a folder with the name `eigen` (e.g. `/lib/eigen`).
3. [Set the environment variable](http://www.computerhope.com/issues/ch000549.htm) `EIGEN_ROOT` to point to that folder location.
4. Unzip Eigen to the folder and then rename the unzipped folder (e.g. `eigen-eigen-da9b4e14c255`) to the name `eigen3`.

## Linux 

On Linux if need a command line way to do this try the following:

````
mkdir ~/eigen3
cd ~/eigen3
wget http://bitbucket.org/eigen/eigen/get/3.3.2.zip
unzip 3.3.2.zip
mv  eigen-eigen-da9b4e14c255 eigen3
rm 3.3.2.zip
echo export EIGEN_ROOT=~/eigen3 >> ~/.bashrc
source  ~/.bashrc
````