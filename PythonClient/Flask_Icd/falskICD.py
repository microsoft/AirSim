import enum
from flask import request
from datetime import datetime
from flask import Flask, Response, render_template, request, jsonify
from flask_socketio import SocketIO, emit, send
import airsim
import numpy as np 
import math
import time
from threading import Thread

import json
app = Flask(__name__)
socketio = SocketIO(app, ping_timeout=100, ping_interval=100)

hot_point_ned_coordinate = []
air_sim = None
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
    position_set = 2
    RotateToYaw = 3
    Gimbal = 4
    Land = 5
    HotPoint = 6
    WayPoints = 7
    HotPointAction = 8


Daytype = {}
Daytype[ICDOperation.TakeOff] = 'takeoff'
Daytype[ICDOperation.position_set] = 'position_set'
Daytype[ICDOperation.Land] = 'land'
Daytype[ICDOperation.RotateToYaw] = 'rotateToYaw'
Daytype[ICDOperation.Gimbal] = 'gimbal'
Daytype[ICDOperation.HotPoint] = 'hotPoint/upload'
Daytype[ICDOperation.WayPoints] = 'wayPoint/uploadComplex'
Daytype[ICDOperation.HotPointAction] = 'hotPoint/action'

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
            import hotPoint 
        elif operation == Daytype[icd_multirotor.wayPoints]:
            import wayPoints   
        elif operation == Daytype[ICDOperation.position_set]:
            import positionSet
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

            thread = Thread(target=takeoff_operation, kwargs={'value': request.args.get('value', operation)})
            thread.start()

            respons = {"success": True, "message": ""}
            return jsonify(respons)
        else:
            print(msg)
            respons = {"success": False, "message": msg}
            return jsonify(respons)


def takeoff_operation(value):
     import takeoff
     from takeoff import Takeoff
     _task = Takeoff(value)
     _task.start()



#   Land            
# ========================================================================== #            
@app.route('/land', methods=['GET', 'POST'])
def land():
    if request.method == "POST":
        import sys
        sys.path.insert(1, '../icd_multirotor')
        thread = Thread(target=land_operation)
        thread.start()
        respons = {"success": True, "message": ""}
        return jsonify(respons)


def land_operation():
    import land
    from land import Land
    _task = Land()
    _task.start()



# HotPoint  
#
# Body:
# {	  "latitude": 32.8004,
#     "longitude": 35.05148,
#     "altitude":22,
#     "radius":20,
#     "angular_speed": 5,
#     "is_clockwise": 0,
#     "start_point": 0,
#     "yaw_mode": 2
#  }
# ========================================================================== #            
# @app.route('/hotPoint/upload', methods=['GET', 'POST'])
# def hotPoint():
#     if request.method == "POST":
#         data = request.get_json()
#         print("request")
#         msg = "data is missing !"
#         # ned_c = geo_to_ned(gps_location)
#         if data:
#             import sys
#             sys.path.insert(1, '../icd_multirotor')
#             coordinates = []
#             coordinates = [data['latitude'],data['longitude'],data['altitude']] ## get add lon\lat
#             ned_coordinate = geo_to_ned(coordinates)
#             thread = Thread(target=hotpoint_operation, kwargs={'value': request.args.get('value', ned_coordinate)})
#             thread.start()
            
#             respons = {"success": True, "message": ""}
#             return jsonify(respons)
#         else:
#             print(msg)
#             respons = {"success": False, "message": msg}
#             return jsonify(respons)


# def hotpoint_operation(value):
#     import hotPoint
#     from hotPoint import HotPoint
#     print(value[0], value[1], value[2])
#     # _task = HotPoint(value[0], value[1], value[2])
#     _task = HotPoint(value[0], value[1], 461)
#     _task.start()
#     #TODO::implement the mission..(orbit?) 

@app.route('/hotPoint/upload', methods=['GET', 'POST'])
def hotPoint():
    if request.method == "POST":
        data = request.get_json()
        print("request")
        msg = "data is missing !"
        # ned_c = geo_to_ned(gps_location)
        if data:
            import sys
            sys.path.insert(1, '../icd_multirotor')
            coordinates = []
            coordinates = [data['latitude'],data['longitude'],data['altitude']] ## get add lon\lat
            global hot_point_ned_coordinate 
            ned_coordinates =  geo_to_ned(coordinates)
            hot_point_ned_coordinate = [ned_coordinates[0],ned_coordinates[1],ned_coordinates[2]]
            #thread = Thread(target=hotpoint_operation, kwargs={'value': request.args.get('value', ned_coordinate)})
            #thread.start()
            print("FFFFFFFFFFFFFFFFFFF")
            print(ned_coordinates)
            respons = {"success": True, "message": ""}
            return jsonify(respons)
        else:
            print(msg)
            respons = {"success": False, "message": msg}
            return jsonify(respons)



# hotPoint Action
#
# Body:
# {	  
#   "action": 0/1/2/3
# }
# ========================================================================== #            
@app.route('/hotPoint/action', methods=['GET', 'POST'])
def hotPointAction():
    if request.method == "POST":
        data = request.get_json()
        msg = "action data is missing !"
        if data:
            import sys
            sys.path.insert(1, '../icd_multirotor')
            action = data['action']
            if (action == 0):
                print(hot_point_ned_coordinate)
                thread = Thread(target=hotpoint_action_operation)
                thread.start() 

            respons = {"success": True, "message": ""}
            return jsonify(respons)
        else:
            print(msg)
            respons = {"success": False, "message": msg}
            return jsonify(respons)


def hotpoint_action_operation():
    import hotPoint
    from hotPoint import HotPoint
    ##global param....
    _task = HotPoint(hot_point_ned_coordinate[0], hot_point_ned_coordinate[1], 461)
    _task.start()




# WayPoints  
# 
# { 
# 	"action_on_finish": 0,
# 	"points": [
#	 	{
#	 		"latitude": 32.922820,
#	 		"longitude": 35.28677496,
#	 		"altitude": 6,
#	 		"velocity": 10,
#	 		"yaw": 90
#	 	},
#	 	{
#	 		"latitude": 32.922020,
#	 		"longitude": 35.28707496,
#	 		"altitude": 6,
#	 		"velocity": 10,
#	 		"yaw": 90
#	 	},
#	 		 	{
#	 		"latitude": 32.921820,
#	 		"longitude": 35.28777496,
#	 		"altitude": 6,
#	 		"velocity": 10,
#	 		"yaw": 90
#	 	}
# 	]
# }
# ========================================================================== #            
@app.route('/wayPoint/uploadComplex', methods=['GET', 'POST'])
def wayPoints():
    if request.method == "POST":
        data = request.get_json()
        print("request")
        msg = "data is missing !"
        if data:
            import sys
            sys.path.insert(1, '../icd_multirotor')

            points = data['points']
            #
            path = []  
            array_length = len(points)
            for i in range(array_length):
                point = points[i] #{X,Y,Z}
                x = point['latitude']
                y = point['longitude']
                z = point['altitude']
                geo_point = []
                geo_point = [x,y,z]
                ned_coordinate = geo_to_ned(geo_point)
                airSimPoint = airsim.Vector3r(ned_coordinate[0],ned_coordinate[0],461)
                print(airSimPoint)
                path.append(airSimPoint)
            #
            thread = Thread(target=waypoint_operation, kwargs={'value': request.args.get('value', path)})
            thread.start()
            
            respons = {"success": True, "message": ""}
            return jsonify(respons)
        else:
            print(msg)
            respons = {"success": False, "message": msg}
            return jsonify(respons)


def waypoint_operation(value):
    import wayPoints
    from wayPoints import WayPoints
    _task = WayPoints(value,80)
    _task.start()



#   position_set
#
#  { 
#	"x": 20,
#	"y": 20,
#	"z": 25,
#	"tolerance": 2
#}                        
# ========================================================================== #  
@app.route('/position_set', methods=['GET', 'POST'])
def positionSet():
    if request.method == "POST":
        data = request.get_json()
        print("request")
        x = data['x']
        y = data['y']
        z = data['z']
        ned_coordinates = [x,y,z]

        msg = "NED is missing"
        if ned_coordinates:
            import sys
            sys.path.insert(1, '../icd_multirotor')

            thread = Thread(target=position_set_operation, kwargs={'value': request.args.get('value', ned_coordinates)})
            thread.start()

            respons = {"success": True, "message": ""}
            return jsonify(respons)
        else:
            print(msg)
            respons = {"success": False, "message": msg}
            return jsonify(respons)


def position_set_operation(value):
    import positionSet
    from positionSet import PositionSet
    _task = PositionSet(value[0], value[1], value[2])
    _task.start()


#   WebSocket -> start !     
# ========================================================================== #          
@app.route('/api/WebSocket/start', methods=['GET'])
def WebSocketStart():
    if request.method == "GET":
        print("GET /WebSocket/start")
        time.sleep(1)
        global air_sim 
        air_sim = init_airsim()
        while True:
            data = load_airsim(air_sim)
            print(data)            
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
        "height_above_takeoff":-(kinematics_estimated.position.z_val) - 441, ## -( kinematics_estimated.position.linear_velocity.z_val),
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
    

############# Socket.io #############
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



def geo_to_ned(gps_location):
    ##air_sim = init_airsim()
    global air_sim
    home_point = air_sim.getHomeGeoPoint()
    print(home_point)
    d_lat = gps_location[0] - home_point.latitude
    d_lon = gps_location[1] - home_point.longitude
    d_alt = home_point.altitude - gps_location[2]


    radian = np.deg2rad(d_lat) 
    x= radian * 6378137.0 # 6378137.0f = earth_radius
    y =  np.deg2rad(d_lon) * 6378137.0 * math.cos( np.deg2rad(gps_location[1]))
    ned_coordinates = []
    ned_coordinates = [x,y,d_alt] 

    print(ned_coordinates[0])
    print(ned_coordinates[1])
    print(ned_coordinates[2])
    return (ned_coordinates)

if __name__ == '__main__':
    app.run(debug=True)
    