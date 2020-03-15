start /d "C:\terrains\Build" AirSimAssets.exe /flaskPortArg=5000  /unityDronePort=41451
start /d "C:\Git\AirSim-1\PythonClient\Flask_Icd" 
set FLASK_APP=falskICD
flask run --host=0.0.0.0 --port=5000