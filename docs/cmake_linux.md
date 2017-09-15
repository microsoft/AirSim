# Installing cmake on Linux

If you don't have cmake version 3.5 (for example, 3.2.2 is the default on Ubuntu 14) you can run the following:

````
mkdir ~/cmake-3.5.1
cd ~/cmake-3.5.1
wget https://cmake.org/files/v3.5/cmake-3.5.1-Linux-x86_64.sh
````

Now you have to run this command by itself (it is interactive)
````
sh cmake-3.5.1-Linux-x86_64.sh --prefix ~/cmake-3.5.1
````

Answer 'n' to the question about creating another cmake-3.5.1-Linux-x86_64 folder and then 
````
sudo update-alternatives --install /usr/bin/cmake cmake ~/cmake-3.5.1/bin/cmake 60
````

Now type `cmake --version` to make sure your cmake version is 3.5.1.