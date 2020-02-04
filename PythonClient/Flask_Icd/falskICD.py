import enum
from flask import request
from datetime import datetime
from flask import Flask, Response, render_template, request, jsonify
from flask_socketio import SocketIO, emit, send
import airsim
import math
import time

import json
app = Flask(__name__)
socketio = SocketIO(app, ping_timeout=100, ping_interval=100)

posts = [{
    'author': "yigal",
    'title': "1",
    'content': "First post content",

}, {
    'author': "pigal",
    'title': "2",
    'content': "Second post content",
}]

@app.route('/')
@app.route('/home')
def home():
    """Renders the home page."""
    return render_template('index.html',
                           posts=posts,
                           title='Contact',
                           year=datetime.now().year,
                           message='Your contact page.')


@app.route('/SomeFunction')
def SomeFunction():
    print('In SomeFunction')
    return "Nothing"

# import requests
# res = requests.post('http://localhost:5000/api/add_message/1234',
# json={"mytext":"lalala"})


@app.route('/button_press')
def button_press():
    print('In SomeFunction')
    return "Nothing"


@app.route('/form', methods=['GET', 'POST'])
def form():
    data = request.get_json()
    return "Nothing"


class ICDOperation(enum.Enum):
    # TakeOff = "takeoff"
    # MoveToPosition = "moveToPosition"
    # Land = "land",
    # RotateToYaw = "rotateToYaw",
    # Gimbal = "gimbal"
    TakeOff = 1
    MoveToPosition = 2
    RotateToYaw = 3
    Gimbal = 4
    Land = 5
    HotPoint = 6


Daytype = {}
Daytype[ICDOperation.TakeOff] = 'takeoff'
Daytype[ICDOperation.MoveToPosition] = 'moveToPosition'
Daytype[ICDOperation.Land] = 'land'
Daytype[ICDOperation.RotateToYaw] = 'rotateToYaw'
Daytype[ICDOperation.Gimbal] = 'gimbal'
Daytype[ICDOperation.HotPoint] = 'hotPoint'


@app.route('/addRegion', methods=['POST'])
def addRegion():

    # return (request.form['projectFilePath'])
    return "Nothing"


@app.route('/ICD/', methods=['GET', 'POST'])
def ICD():
    if request.method == "POST":
        data = request.get_json()
        print("request")
        operation = data['operation']
        print(operation)
        import sys
      #  sys.path.insert(1, 'D:\Git\AirSim\PythonClient\icd_multirotor')
        sys.path.insert(1, '../icd_multirotor')
        if operation == Daytype[ICDOperation.TakeOff]:
            import takeoff
        elif operation == Daytype[ICDOperation.Land]:
            print("land action") 
            import land
        elif operation == Daytype[ICDOperation.HotPoint]:
            print("###################### hotPoint ###################")   
            import hotPoint 
        elif operation == Daytype[ICDOperation.MoveToPosition]:
            coordinates = data['coordinates']
            import moveToPosition
            from moveToPosition import MoveToPosition
            r = MoveToPosition(
                coordinates[0], coordinates[1], coordinates[2])
            r.start()
        elif operation == Daytype[ICDOperation.RotateToYaw]:
            import rotateToYaw
            from rotateToYaw import RotateToYaw
            angle = data['angle']
            print(angle)
            r = RotateToYaw(angle)
            r.start()
        elif operation == Daytype[ICDOperation.Gimbal]:
            import gimbal
        return render_template('index.html')


#   Takeoff            
# ========================================================================== #  
@app.route('/takeoff', methods=['GET', 'POST'])
def takeoff():
    if request.method == "POST":
        data = request.get_json()
        print("request")
        operation = data['operationalAlt']
        msg = "missing Alt operand"
        if operation:
            import sys
            sys.path.insert(1, '../icd_multirotor')
            import takeoff
            from takeoff import Takeoff
            print(data)
            r2 = Takeoff(operation)
            result = r2.start()
            if result == True:

                respons = {"success": True, "message": ""}
                return jsonify(respons)
            else:
                msg = "got error as collision"
                respons = {"success": False, "message": msg}
                return jsonify(respons)
        else:
            print(msg)
            respons = {"success": False, "message": msg}
            return jsonify(respons)

#   Land            
# ========================================================================== #            
@app.route('/land', methods=['GET', 'POST'])
def land():
    if request.method == "POST":
        import sys
        sys.path.insert(1, '../icd_multirotor')
        import land
        from land import Land
        r2 = Land()
        result = r2.start()
        if result == True:
            respons = {"success": True, "message": ""}
            return jsonify(respons)
        else:
            msg = "got error as collision"
            respons = {"success": False, "message": msg}
            return jsonify(respons)


# HotPoint  
# WebSocket -> start ! 
#
# Body:
# { "latitude": 20,
#   "longitude":21,
#   "altitude":22,
#   "radius":20,
#   "is_clockwise":20,
#   "start_point":20,
#   "yaw_mode":20
#  }            
# ========================================================================== #            
@app.route('/hotPoint', methods=['GET', 'POST'])
def hotPoint():
    if request.method == "POST":
        data = request.get_json()
        print("request")
        msg = "data is missing !"
        if data:
            import sys
            sys.path.insert(1, '../icd_multirotor')

            coordinates = []
            coordinates = [data['latitude'],data['longitude'],data['altitude']]
            import hotPoint
            from hotPoint import HotPoint
            task = HotPoint(coordinates[0], coordinates[1], coordinates[2])
            result = task.start()
            if result == True:
                respons = {"success": True, "message": ""}
                return jsonify(respons)
            else:
                msg = "got error as collision"
                respons = {"success": False, "message": msg}
                return jsonify(respons)
        else:
            print(msg)
            respons = {"success": False, "message": msg}
            return jsonify(respons)


#   WebSocket -> start !     
# ========================================================================== #          
@app.route('/api/WebSocket/start', methods=['GET'])
def WebSocketStart():
    if request.method == "GET":
        print("GET /WebSocket/start")
        time.sleep(1)
        air_sim = init_airsim()
        while True:
            data = load_airsim(air_sim)
            print(data)            
            # print("")
            socketio.emit('my', data, broadcast=True)
            time.sleep(1)
        respons = {"success": True, "message": "WebSocket start"}
        return jsonify(respons)

#   initialize the client.          
# ========================================================================== #    
def init_airsim():
    airsim_client = airsim.MultirotorClient()
    airsim_client.confirmConnection()
    airsim_client.enableApiControl(True)
    airsim_client.armDisarm(True)
    return airsim_client

#   load telmetry          
# ========================================================================== #   
def load_airsim(airsim_client):
    rpcinfo = airsim_client.getMultirotorState()
    gps_location = rpcinfo.gps_location
    kinematics_estimated = rpcinfo.kinematics_estimated
    pitch, roll, yaw = airsim.to_eularian_angles(
        rpcinfo.kinematics_estimated.orientation)
    homepoint = airsim_client.getHomeGeoPoint()

    telemetry = {
        "battery_state": {
            "percentage": 70.04
        },
        "distance_from_home": 1561.4,
        "gimbal": {
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw
        },
        "height_above_takeoff":-(kinematics_estimated.position.z_val), ## -( kinematics_estimated.position.linear_velocity.z_val),
        "gps_health": 5,
        "heading": math.degrees(yaw),
        "velocity": {
            "x": kinematics_estimated.linear_velocity.x_val,
            "y": kinematics_estimated.linear_velocity.y_val,
            "z": kinematics_estimated.linear_velocity.z_val
        },
        "gps_position": {
            "latitude": gps_location.latitude,
            "altitude": gps_location.altitude,
            "longitude": gps_location.longitude
        },
        "last_change_time": rpcinfo.timestamp,
        "lastHome": {
            "latitude": homepoint.latitude,
            "operationalAlt": homepoint.altitude,
            "longitude": homepoint.longitude
        },
        "owner": "droneService",
        "state": {
            "armed": True
        },
        "wayPoints": {
            "status": 2
        },
        "keepAlive": rpcinfo.timestamp
    }
    return telemetry


#   WebSocket -> end !            
# ========================================================================== #   
@app.route('/api/WebSocket/end', methods=['GET'])
def WebSocketEnd():
    if request.method == "GET":
        print("GET /WebSocket/end")
        import sys
        sys.path.insert(1, '../webSocket')
        import wsClientLoop
        from wsClientLoop import WebSocketClient
        wbs = WebSocketClient()
        result = wbs.end()
        respons = {"success": True, "message": "WebSocket end"}
        return jsonify(respons)

# ========================================================================== #   
# ############################# Socket.io ################################## #         
# ========================================================================== #   
@socketio.on('connect')
def WSocketConnect():
    print('connect')


@socketio.on('disconnect')
def WSocketDisconnect():
    print('disconnect')


@socketio.on('keepAlive')
def WSocketHandleKeepAlive(json):
    # print('received keepAlive: ' + str(json))
    pass


@socketio.on('my')
def handle_my_custom_event(json):
    print('received my: ' + str(json))


@socketio.on('force_send')
def handle_force_send(json):
    print('received force_send: ' + str(json))


@socketio.on('force_stop')
def handle_force_stop(json):
    print('received force_stop: ' + str(json))
    

if __name__ == '__main__':
    app.run(debug=True)
    