@echo off
more +1 C:\Git\AirSim-1\PythonClient\Flask_Icd\instance\config.cfg > C:\Git\AirSim-1\PythonClient\Flask_Icd\instance\config.cfg
echo SESSION_COOKIE_PATH=41451 >> C:\Git\AirSim-1\PythonClient\Flask_Icd\instance\config.cfg

start  "C:\Git\AirSim-1\PythonClient\Flask_Icd" 
set FLASK_APP=falskICD
flask run --host=0.0.0.0 --port=5000