import time
from pymavlink import mavutil

# connect to the mavlink to get pitch and roll
# gonna have to send velocity commands too
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
master.mav.request_data_stream_send(1, 30, mavutil.mavlink.MAV_DATA_STREAM_ALL, 30, 1)

# to use new commands:
# every command is master.mav.<msg name>_send()

# how to arm/disarm
master.arducopter_arm()
# master.arducopter_disarm()

# how to change flight mode:
mode_id = master.mode_mapping()['GUIDED']
master.mav.set_mode_send(
	0, # target sys
	mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
	mode_id)
while True:
	# wait for ack
	ack = master.recv_match(type='COMMAND_ACK', blocking=True)
	ack = ack.to_dict()
	
	if ack['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
		continue
	print(mavutil.mavlink.enums['MAV_RESULT'][ack['result']].description)
	break
	
print('worked')

# note - z velocity will be inverted (NED coordinates)
while True:
	time.sleep(0.25)
	master.mav.set_position_target_local_ned_send(0, 0, 0,
				mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
				0b0000111111000111,
				0, 0, 0,
				0.696969, 0, 0,
				0, 0, 0,
				0, 0)

'''
msg = master.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0, # target sys, component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
		0b0000111111000111, # type mask
		0, 0, 0, # xyz pos goal (unused)
		vx, vy, vz, # xyz velocities in m/s
		0, 0, 0, # xyz accel (unused)
		0, 0) # yaw rate
		
while True:
	master.send_mavlink(msg)
	print("sent vel cmd:", msg)
	time.sleep(0.1)
'''	

'''
while True:
	msg = master.recv_match()
	
	if not msg:
		continue
	
	print(msg.get_type())
	if msg.get_type() == 'ATTITUDE':
		roll = msg.to_dict()['roll']*180/3.1415
		pitch = msg.to_dict()['pitch']*180/3.1415
		print(msg.to_dict())
'''
