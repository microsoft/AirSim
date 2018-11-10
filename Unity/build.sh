if [ ! -f ../external/rpclib/rpclib-2.2.1/rpclib.pc.in ]; then
    >&2 echo "error, rpc.pc.in not found"
fi
cp ../external/rpclib/rpclib-2.2.1/rpclib.pc.in AirLibWrapper/AirsimWrapper/rpclib.pc.in

if [ ! -d "linux-build" ]; then
    mkdir linux-build;
fi

cd linux-build;

cmake ../AirLibWrapper/AirsimWrapper;
make;

if [ ! -d "../UnityDemo/Assets/Plugins" ]; then
	mkdir ../ UnityDemo/Assets/Plugins;
fi
cp libAirsimWrapper.so ../UnityDemo/Assets/Plugins;
cd ..;
#rm -r linux-build;
rm AirLibWrapper/AirsimWrapper/rpclib.pc.in
