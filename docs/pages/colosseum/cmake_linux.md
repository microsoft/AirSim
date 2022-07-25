# Installing cmake on Linux

If you don't have cmake version 3.10 (for example, 3.2.2 is the default on Ubuntu 14) you can run the following:

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