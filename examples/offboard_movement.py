#!/usr/bin/env python3
# Basic move routine using offboard

import asyncio
from contextlib import aclosing

from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync, ActionError
from mavsdk.asyncio.plugins.offboard import (
    OffboardAsync,
    OffboardError,
    VelocityBodyYawspeed,
)
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
        offboard = OffboardAsync(drone)
        telemetry = TelemetryAsync(drone)

        print("Waiting for position and home estimates...")
        async with aclosing(telemetry.subscribe_health()) as health_updates:
            async for health in health_updates:
                if health.is_global_position_ok and health.is_home_position_ok:
                    break

        # An initial setpoint is required before starting offboard mode.
        await offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        )

        try:
            print("Arming...")
            await action.arm()
        except ActionError as error:
            print(f"Unable to arm, not ready: {error}")
            return

        offboard_started = False
        try:
            await offboard.start()
            offboard_started = True

            # Body-frame down velocity is negative when climbing.
            await offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, -1.0, 0.0)
            )
            print("Taking off upwards...")
            await asyncio.sleep(15)

            await offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
            await asyncio.sleep(10)

            print("Flying in a circle...")
            await offboard.set_velocity_body(
                VelocityBodyYawspeed(5.0, 0.0, 0.0, 30.0)
            )
            await asyncio.sleep(15)

            await offboard.set_velocity_body(
                VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
            )
        finally:
            if offboard_started:
                try:
                    await offboard.stop()
                except OffboardError as error:
                    print(f"Failed to stop offboard mode: {error}")
            print("Landing...")
            await action.land()


if __name__ == "__main__":
    asyncio.run(run())
