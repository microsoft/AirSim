# Installing cmake on Linux

If you have Ubuntu 18.04 you can get a good version of cmake by doing this:
```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get -y update
sudo apt-get install cmake
```

If you want to manually install a specific version of cmake you can do this on any version of Ubuntu:

```
mkdir ~/cmake-3.10.2
cd ~/cmake-3.10.2
wget https://cmake.org/files/v3.10/cmake-3.10.2-Linux-x86_64.sh
```

Now you have to run this command by itself (it is interactive)
```
sh cmake-3.10.2-Linux-x86_64.sh --prefix ~/cmake-3.10.2
```

Answer 'n' to the question about creating another cmake-3.10.2-Linux-x86_64 folder and then 
```
sudo update-alternatives --install /usr/bin/cmake cmake ~/cmake-3.10.2/bin/cmake 60
```

Now type `cmake --version` to make sure your cmake version is 3.10.2.