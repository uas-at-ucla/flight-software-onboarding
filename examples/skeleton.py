#!/usr/bin/env python3
# Skeleton for flight software routine

import asyncio

from mavsdk.asyncio import ComponentType, Configuration, Mavsdk


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

        # Create the plugins needed for your routine, e.g. ActionAsync(drone).


if __name__ == "__main__":
    asyncio.run(run())
