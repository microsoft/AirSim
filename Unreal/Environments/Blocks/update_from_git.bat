robocopy /MIR ..\..\Plugins\AirSim Plugins\AirSim /XD temp *. /njh /njs /ndl /np
robocopy /MIR ..\..\..\AirLib Plugins\AirSim\Source\AirLib /XD temp *. /njh /njs /ndl /np
CALL clean.bat
CALL GenerateProjectFiles.bat

pause