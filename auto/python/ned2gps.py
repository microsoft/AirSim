import math    

EARTH_RADIUS = 6378137.0

home_lat = 40
home_lon = 100

home_alt = -21396.533203
#player_start
ps_z = 0
 
#destination
dest_x = 59766.203125
dest_y = 112109.476562
z = 20720.0/100

x = (dest_x)/100
y = (dest_y)/100


x_rad = x/EARTH_RADIUS
y_rad = y/EARTH_RADIUS

c = math.sqrt(x_rad*x_rad + y_rad*y_rad)
sin_c = math.sin(c)
cos_c = math.cos(c)

home_lat_rad = home_lat * math.pi / 180
home_lon_rad = home_lon * math.pi / 180
home_cos_lat = math.cos(home_lat_rad);
home_sin_lat = math.sin(home_lat_rad);

lat_rad = math.asin(cos_c * home_sin_lat + (x_rad * sin_c * home_cos_lat) / c)
lon_rad = (home_lon_rad + math.atan2(y_rad * sin_c, c * home_cos_lat * cos_c - x_rad * home_sin_lat * sin_c))

print(lat_rad*180/math.pi,lon_rad*180/math.pi, ps_z-z)
