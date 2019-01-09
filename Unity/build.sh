if [ ! -f ../external/rpclib/rpclib-2.2.1/rpclib.pc.in ]; then
    >&2 echo "error, rpc.pc.in not found"
fi
cp ../external/rpclib/rpclib-2.2.1/rpclib.pc.in AirLibWrapper/AirsimWrapper/rpclib.pc.in

if [ ! -d "linux-build" ]; then
    mkdir linux-build;
fi


cd linux-build;
export CC="clang-5.0"
export CXX="clang++-5.0"
CMAKE="$(readlink -f ../../cmake_build/bin/cmake)"

"$CMAKE" ../../cmake ../AirLibWrapper/AirsimWrapper;
make -j`nproc`;
if [ ! -d "../UnityDemo/Assets/Plugins" ]; then
	mkdir ../ UnityDemo/Assets/Plugins;
fi
cp libAirsimWrapper.so ../UnityDemo/Assets/Plugins;

cd ..
if [ -f AirLibWrapper/AirsimWrapper/rpclib.pc.in ]; then
	rm AirLibWrapper/AirsimWrapper/rpclib.pc.in
fi
