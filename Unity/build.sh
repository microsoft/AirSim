if [ ! -f ../external/rpclib/rpclib-2.2.1/rpclib.pc.in ]; then
    >&2 echo "error, rpc.pc.in not found"
fi
cp ../external/rpclib/rpclib-2.2.1/rpclib.pc.in AirLibWrapper/AirsimWrapper/rpclib.pc.in

if [ ! -d "linux-build" ]; then
    mkdir linux-build;
fi

cd linux-build;
export CC=/usr/bin/clang-5.0;
export CXX=/usr/bin/clang++-5.0;
cmake ../AirLibWrapper/AirsimWrapper;
make;
cd ..;
if [ -f AirLibWrapper/AirsimWrapper/rpclib.pc.in ]; then
	rm AirLibWrapper/AirsimWrapper/rpclib.pc.in
fi
