#!/usr/bin/env python3
# Take off and land routine

import asyncio
from contextlib import aclosing

from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync, ActionError
from mavsdk.asyncio.plugins.telemetry import TelemetryAsync


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
        telemetry = TelemetryAsync(drone)

        print("Waiting for position and home estimates...")
        async with aclosing(telemetry.subscribe_health()) as health_updates:
            async for health in health_updates:
                if health.is_global_position_ok and health.is_home_position_ok:
                    break

        await action.set_takeoff_altitude(5.0)

        try:
            print("Arming...")
            await action.arm()
        except ActionError as error:
            print(f"Failed to arm: {error}")
            return

        try:
            print("Taking off...")
            await action.takeoff()
            await asyncio.sleep(10)
        finally:
            print("Landing...")
            await action.land()


if __name__ == "__main__":
    asyncio.run(run())
