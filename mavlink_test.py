
from pymavlink import mavutil

# connect to the mavlink to get pitch and roll
# gonna have to send velocity commands too
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
#master.mav.request_data_stream_send(1, 30, mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER, 30, 1)
master.mav.request_data_stream_send(1, 30, mavutil.mavlink.MAV_DATA_STREAM_ALL, 30, 1)

while True:
	msg = master.recv_match()
	
	if not msg:
		continue
	
	print(msg.get_type())
	if msg.get_type() == 'ATTITUDE':
		roll = msg.to_dict()['roll']*180/3.1415
		pitch = msg.to_dict()['pitch']*180/3.1415
		print(msg.to_dict())
