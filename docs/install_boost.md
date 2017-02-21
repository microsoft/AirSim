# Installing Boost

1. Download Boost: [Boost 1.63 zip file](https://sourceforge.net/projects/boost/files/boost/1.63.0/boost_1_63_0.zip/download).
2. Unzip Boost to a new folder and then [set the environment variable](http://www.computerhope.com/issues/ch000549.htm) `BOOST_ROOT` to point to that folder location (for example `boost_1_63_0`).
3. Open a VS2015 x64 Native Command Prompt, `cd %BOOST_ROOT%`, and run `bootstrap.bat`.
4. From that same location run 
`b2 variant=debug,release link=static runtime-link=shared threading=multi address-model=64`
and wait about 20 minutes (coffee time :-). Note: On Linux, you can drop the `address-model=64` and just build release `variant=release`.


## Linux 

On Linux if you need a command line way to do this try the following:

````
wget https://sourceforge.net/projects/boost/files/boost/1.63.0/boost_1_63_0.zip/download
unzip download
rm boost_1_63_0.zip
echo export BOOST_ROOT=$PWD/boost_1_63_0 >> ~/.bashrc
source ~/.bashrc
cd boost_1_63_0
./bootstrap.sh
./b2 variant=release link=static runtime-link=shared threading=multi
````

If you want to debug the boost binaries use variant=debug instead.