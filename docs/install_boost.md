# Installing Boost

1. Download [boost 1.63 zip file](https://sourceforge.net/projects/boost/files/boost/1.63.0/boost_1_63_0.zip/download) 
2. Unzip boost someplace and then [set an environment variable](http://www.computerhope.com/issues/ch000549.htm) named `BOOST_ROOT` that points to the folder you just unzipped.  It should point to the root folder that contains 'bootstrap.bat'.
3. Open VS2015 x64 Native Command Prompt and cd over to the boost folder and run `bootstrap.bat`
4. from that same location run **b2 variant=debug,release link=static runtime-link=shared threading=multi address-model=64** and wait about 20 minutes (coffee time :-). Note: On Linux you can drop the address-model=64.
