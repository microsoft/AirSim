# An Orbit Trajectory

Moved here from [https://github.com/microsoft/AirSim/wiki/An-Orbit-Trajectory](https://github.com/microsoft/AirSim/wiki/An-Orbit-Trajectory)

Have you ever wanted to fly a nice smooth circular orbit? This can be handy for capturing 3D objects from all sides especially if you get multiple orbits at different altitudes.

So the `PythonClient/multirotor` folder contains a script named [Orbit](https://github.com/microsoft/AirSim/blob/main/PythonClient/multirotor/orbit.py) that will do precisely that.

See [demo video](https://youtu.be/RFG5CTQi3Us)

The demo video was created by running this command line:

```shell
python orbit.py --radius 10 --altitude 5 --speed 1 --center "0,1" --iterations 1
```

This flies a 10 meter radius orbit around the center location at (startpos + radius * [0,1]), in other words, the center is located `radius` meters away in the direction of the provided center vector.  It also keeps the front-facing camera on the drone always pointing at the center of the circle. If you watch the flight using LogViewer you will see a nice circular pattern get traced out on the GPS map:

![image](images/orbit.png)

The core of the algorithm is not that complicated.  At each point on the circle, we look ahead by a small delta in degrees, called the `lookahead_angle`, where that angle is computed based on our desired velocity.  We then find that lookahead point on the circle using sin/cosine and make that our "target point". Calculating the velocity then is easy, just subtract our current position from that point and feed that into the AirSim method `moveByVelocityZ`.
