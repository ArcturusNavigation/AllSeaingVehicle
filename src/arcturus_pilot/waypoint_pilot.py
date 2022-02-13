#!/usr/bin/env python

import rospy 
from std_msgs.msg import String
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
import asyncio 
# Based on https://github.com/mavlink/MAVSDK-Python/blob/main/examples/telemetry_takeoff_and_land.py

async def run():
    boat = System()
    print('Attempting to connect to boat...')
    await boat.connect(system_address='serial:///dev/ttyTHS2:921600')
    async for state in boat.core.connection_state():
        if state.is_connected:
            print('Connected to boat!')
            break
    
    print_gps_task = asyncio.ensure_future(print_gps(boat))
    print_imu_task = asyncio.ensure_future(print_imu(boat))
    print_heading_task = asyncio.ensure_future(print_heading(boat))
    print_health_task = asyncio.ensure_future(print_health(boat))
    print_battery_task = asyncio.ensure_future(print_battery(boat))
    running_tasks = [print_gps_task, print_imu_task, print_heading_task, print_health_task, print_battery_task]

    # print("Arming boat...")
    # await boat.action.arm()
    # print("Armed boat!")

    # print("Waiting 5 seconds...")
    # await asyncio.sleep(5)
    # print("Waited 5 seconds!")

    # print("Disarming boat...")
    # await boat.action.disarm()
    # print("Dismared boat!")
    
async def print_gps(boat):
    """Prints the GPS location when it changes"""
    previous_gps_info = None
    
    async for gps_info in boat.telemetry.gps_info():
        if gps_info != previous_gps_info:
            previous_gps_info = gps_info
            print(f"GPS info: {gps_info}")
    
async def print_imu(boat):
    """Prints the GPS location when it changes"""
    
    previous_imu = None
    async for imu in boat.telemetry.imu():
        if imu != previous_imu:
            previous_imu = imu
            print(f"IMU: {imu}")

async def print_heading(boat):
    """Prints the heading of the boat when it changes"""
    previous_heading = None
    
    async for heading in boat.telemetry.heading():
        if heading != previous_heading:
            previous_heading = heading
            print(f"Heading: {heading}")

async def print_health(boat):
    """Prints the health of the various sensors when it changes"""
    previous_health = None

    async for health in boat.telemetry.health():
        if health != previous_health:
            previous_health = health
            print(f"Health: {health}")

async def print_battery(boat):
    """Prints the battery when it changes"""

    previous_battery_remaining_percent = None
    async for battery in boat.telemetry.battery():
        if battery.remaining_percent != previous_battery_remaining_percent:
            previous_battery_remaining_percent = battery.remaining_percent
            print(f"Battery: {battery.remaining_percent}")

def make_mission_item(latitude, longitude, is_fly_through=True, speed=float('nan'), loiter_time=float('nan'), acceptance_radius=float('nan')):
    """Creates a mission item for a mission plan based on given parameters"""
    # Not sure if relative_altitude should be 0 or float('nan')
    # Not sure if speed = -1 or float('nan') makes the speed not change between between waypoints?
    # Documentation: http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/plugins/mission.html

    return MissionItem(latitude, longitude, float('nan'), speed, is_fly_through, float('nan'), float('nan'), 
        MissionItem.CameraAction.NONE, loiter_time, float('nan'), acceptance_radius, float('nan'), float('nan'))

async def wait_mission_complete(boat):
    """"Function that returns only when the boat has finished its current mission"""
    previous_mission_progress = None

    async for mission_progress in boat.mission.mission_progress():
        if mission_progress.current == total:
            return
        if mission_progress != previous_mission_progress:
            previous_mission_progress = mission_progress
            print(f"Mission progress: {mission_progress.current}/{mission_progress.total}")
        
async def follow_waypoints(boat, waypoints):
    """Follows a series of waypoints given as [latitude, longitude] waypoints and waits until it ends"""
    mission_items = []

    for i, (latitude, longitude) in enumerate(waypoints):
        mission_items.append(make_mission_item(latitude, longitude, i == len(waypoints) - 1))

    mission_plan = MissionPlan(mission_items)

    print('Uploading mission to boat...')
    await boat.mission.upload_mission(mission_plan)
    print('Uploaded mission to boat!')

    print('Starting mission...')
    await boat.mission.start_mission()
    print('Started mission!')

    print('Waiting for mission to end...')
    termination_task = asyncio.ensure_future(wait_mission_complete(boat))
    await termination_task
    print('Mission ended!')

def main():
    rospy.init_node('pilot', anonymous= True)
    asyncio.ensure_future(run())
    asyncio.get_event_loop().run_forever()
    rate = rospy.Rate(10) #Frequency in Hz at which we can update next waypoint
    while not rospy.is_shutdown():
        # Check topic and send waypoint command 
        rate.sleep()

if __name__ == '__main__':
    main()
