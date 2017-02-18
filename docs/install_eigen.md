# Install Eigen

Installing Eigen library is easy!

1. Get at least 3.3.x release such as [3.3.2](http://bitbucket.org/eigen/eigen/get/3.3.2.zip).
2. Unzip somewhere on your drive and create environment variable named `EIGEN_ROOT` with value equal to path of the folder. 
If the unzipped folder is named `eigen-eigen-da9b4e14c255` rename that to `eigen3` because the folder
that EIGEN_ROOT points to should contain a single `eigen3` folder and all the eigen source should be inside that.
So you should have something like this:
````
d:/opensouce/eigen-3.3.2        <--- EIGEN_ROOT
└───eigen3   
    └───.hgeol
        .hgignore
        .hgtags
        .hg_archival.txt
        bench
        blas
        cmake
        CMakeLists.txt
        COPYING.BSD
        COPYING.GPL
        COPYING.LGPL
        COPYING.MINPACK
        COPYING.MPL2
        COPYING.README
        CTestConfig.cmake
        CTestCustom.cmake.in
        debug
        demos
        doc
        Eigen
        eigen3.pc.in
        failtest
        INSTALL
        lapack
        README.md
        scripts
        signature_of_eigen3_matrix_library
        test
````        
