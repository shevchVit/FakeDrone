import time

from pioneer_sdk import Pioneer

drone = Pioneer(ip="127.0.0.1", mavlink_port=5656)
drone.arm()
time.sleep(2)
drone.takeoff()
time.sleep(2)
for i in range(10):
    print(drone.get_local_position_lps())
    time.sleep(0.5)
drone.land()
time.sleep(2)
drone.disarm()
time.sleep(2)
