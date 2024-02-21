import os
import time
from threading import Thread
import random
import argparse

os.environ["MAVLINK20"] = '1'

from pymavlink import mavutil


class Drone:
    def __init__(self, ip="127.0.0.1", port=5656, protocol="udpin"):
        self.ip = ip
        self.port = port
        self.protocol = protocol
        self.socket = mavutil.mavlink_connection(f"{self.protocol}:{self.ip}:{self.port}")
        self.boot_timestamp = time.time()
        self.commands_long_apply = {}
        self.last_heartbeat = 0

        self.heartbeat_thread = Thread(target=self.__heartbeat, daemon=True)
        self.telemetry_thread = Thread(target=self.__telemetry_sender, daemon=True)
        # self.message_thread = Thread(target=self.__message_handler, daemon=True)

        self.__init_commands()

    def run(self):
        self.socket.wait_heartbeat()
        self.heartbeat_thread.start()
        self.telemetry_thread.start()
        # self.message_thread.start()
        # self.message_thread.join()
        while True:
            self.__message_handler()

    def __init_commands(self):
        self.commands_long_apply[mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM] = self.__arm_disarm_apply
        self.commands_long_apply[mavutil.mavlink.MAV_CMD_NAV_TAKEOFF] = self.__takeoff_apply
        self.commands_long_apply[mavutil.mavlink.MAV_CMD_NAV_LAND] = self.__land_apply

    def __heartbeat(self):
        while True:
            self.socket.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                           mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(0.05)

    def __message_handler(self):
        while True:
            message = self.socket.recv_msg()
            if message:
                message_type = message.get_type()
                message_d = message.to_dict()
                if message_type == "COMMAND_LONG":
                    command_id = message_d["command"]
                    try:
                        self.commands_long_apply[command_id](message_d)
                        self.socket.mav.command_ack_send(command=command_id,
                                                         result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                                                         result_param2=0)
                    except KeyError:
                        self.socket.mav.command_ack_send(command=command_id,
                                                         result=mavutil.mavlink.MAV_RESULT_UNSUPPORTED,
                                                         result_param2=0)
                elif message_type == "HEARTBEAT":
                    self.last_heartbeat = time.time()
                    # self.socket.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    #                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def __telemetry_sender(self):
        while True:
            timestamp_since_boot = int(time.time() - self.boot_timestamp) * 1000
            x = float(random.randint(-100, 100)) / 10
            y = float(random.randint(-100, 100)) / 10
            z = float(random.randint(0, 100)) / 10
            self.socket.mav.local_position_ned_send(time_boot_ms=timestamp_since_boot,
                                                    x=x, y=y, z=z, vx=0, vy=0, vz=0)
            time.sleep(0.1)

    def __arm_disarm_apply(self, message):
        is_arm = message["param1"]
        if is_arm > 0.0:
            print("ARM")
        else:
            print("DISARM")

    def __takeoff_apply(self, message):
        print("TAKEOFF")

    def __land_apply(self, message):
        print("LAND")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", dest="port", type=int, help="Порт mavlink", default=5656)
    drone = Drone(port=parser.parse_args().port)
    drone.run()

