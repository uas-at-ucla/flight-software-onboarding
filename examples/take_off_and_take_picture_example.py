#!/usr/bin/env python3

import asyncio
from contextlib import aclosing
from pathlib import Path

import cv2
from camera import Video
from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync
from mavsdk.asyncio.plugins.gimbal import (
    ControlMode,
    GimbalAsync,
    GimbalMode,
    SendMode,
)
from mavsdk.asyncio.plugins.telemetry import TelemetryAsync


async def first_gimbal_id(gimbal):
    async with aclosing(gimbal.subscribe_gimbal_list()) as gimbal_updates:
        async for gimbal_list in gimbal_updates:
            if gimbal_list.gimbals:
                return gimbal_list.gimbals[0].gimbal_id


async def next_frame(camera):
    while True:
        frame = camera.frame()
        if frame is not None:
            return frame
        await asyncio.sleep(0.1)


async def run():
    configuration = Configuration.create_with_component_type(
        ComponentType.GROUND_STATION
    )
    async with Mavsdk(configuration) as mavsdk:
        await mavsdk.add_any_connection("udpin://0.0.0.0:14540")
        print("Waiting for drone to connect...")
        drone = await mavsdk.first_autopilot(timeout_s=10.0)
        if drone is None:
            raise RuntimeError("No autopilot found; check that PX4 SITL is running.")
        print("-- Connected to drone!")

        action = ActionAsync(drone)
        gimbal = GimbalAsync(drone)
        telemetry = TelemetryAsync(drone)

        print("Waiting for position and home estimates...")
        async with aclosing(telemetry.subscribe_health()) as health_updates:
            async for health in health_updates:
                if health.is_global_position_ok and health.is_home_position_ok:
                    break

        print("Waiting for a gimbal...")
        gimbal_id = await asyncio.wait_for(first_gimbal_id(gimbal), timeout=10.0)

        altitude = 1.5
        print(f"Setting takeoff altitude to {altitude} m")
        await action.set_takeoff_altitude(altitude)

        image_location = Path(__file__).resolve().parents[1] / "images" / "image.jpg"
        image_location.parent.mkdir(parents=True, exist_ok=True)
        camera = Video()
        armed = False
        controlling_gimbal = False
        try:
            print("Arming drone...")
            await action.arm()
            armed = True
            await action.takeoff()
            await asyncio.sleep(10)

            print("Pointing camera gimbal straight down")
            await gimbal.take_control(gimbal_id, ControlMode.PRIMARY)
            controlling_gimbal = True
            await gimbal.set_angles(
                gimbal_id=gimbal_id,
                roll_deg=0.0,
                pitch_deg=-90.0,
                yaw_deg=0.0,
                gimbal_mode=GimbalMode.YAW_LOCK,
                send_mode=SendMode.ONCE,
            )
            await asyncio.sleep(5)

            print("Taking photo")
            frame = await asyncio.wait_for(next_frame(camera), timeout=10.0)
            if not cv2.imwrite(str(image_location), frame):
                raise RuntimeError(f"Could not save photo to {image_location}")
            print(f"Saved photo to {image_location}")
        finally:
            try:
                if armed:
                    print("Landing...")
                    await action.land()
            finally:
                try:
                    if controlling_gimbal:
                        await gimbal.release_control(gimbal_id)
                finally:
                    camera.stop()


if __name__ == "__main__":
    asyncio.run(run())

