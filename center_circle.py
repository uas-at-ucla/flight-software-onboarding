#!/usr/bin/env python3

import asyncio
import time
from contextlib import aclosing
from pathlib import Path

import cv2

from examples.camera import Video
from mavsdk.asyncio import ComponentType, Configuration, Mavsdk
from mavsdk.asyncio.plugins.action import ActionAsync
from mavsdk.asyncio.plugins.gimbal import (
    ControlMode,
    GimbalAsync,
    GimbalMode,
    SendMode,
)
from mavsdk.asyncio.plugins.offboard import (
    OffboardAsync,
    OffboardError,
    VelocityBodyYawspeed,
)
from mavsdk.asyncio.plugins.telemetry import TelemetryAsync

from vision import find_red_object


TAKEOFF_ALTITUDE_M = 4.0

# Start slowly. Increase only after confirming that movement is correct.
MAX_HORIZONTAL_SPEED_M_S = 0.6

# How close the circle must be to the image center.
CENTER_TOLERANCE_PX = 10

# Stay centered for this long before landing.
CENTERED_TIME_S = 3

# Land if the controller cannot finish within this time.
MAX_CONTROL_TIME_S = 90.0

# Initial guesses for the camera orientation.
# Change either one to -1.0 if that axis moves in the wrong direction.
IMAGE_X_TO_FORWARD_SIGN = 1.0
IMAGE_Y_TO_RIGHT_SIGN = 1.0


class PID:
    """Small PID controller with output and integral limits."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float,
        integral_limit: float = 1.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.previous_error = None

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None

    def update(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        self.integral += error * dt
        self.integral = max(
            -self.integral_limit,
            min(self.integral, self.integral_limit),
        )

        if self.previous_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.previous_error) / dt

        self.previous_error = error

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        return max(
            -self.output_limit,
            min(output, self.output_limit),
        )


async def first_gimbal_id(gimbal: GimbalAsync):
    """Wait for the first available gimbal and return its ID."""

    async with aclosing(gimbal.subscribe_gimbal_list()) as updates:
        async for gimbal_list in updates:
            if gimbal_list.gimbals:
                return gimbal_list.gimbals[0].gimbal_id


async def send_zero_velocity(offboard: OffboardAsync) -> None:
    """Tell the aircraft to hold its current horizontal position."""

    await offboard.set_velocity_body(
        VelocityBodyYawspeed(
            forward_m_s=0.0,
            right_m_s=0.0,
            down_m_s=0.0,
            yawspeed_deg_s=0.0,
        )
    )


async def run() -> None:
    configuration = Configuration.create_with_component_type(
        ComponentType.GROUND_STATION
    )

    async with Mavsdk(configuration) as mavsdk:
        await mavsdk.add_any_connection("udpin://0.0.0.0:14540")

        print("Waiting for drone to connect...")
        drone = await mavsdk.first_autopilot(timeout_s=30.0)

        if drone is None:
            raise RuntimeError(
                "No autopilot found; check that PX4 SITL is running."
            )

        print("Connected to drone")

        action = ActionAsync(drone)
        telemetry = TelemetryAsync(drone)
        offboard = OffboardAsync(drone)
        gimbal = GimbalAsync(drone)

        print("Waiting for position and home estimates...")

        async with aclosing(
            telemetry.subscribe_health()
        ) as health_updates:
            async for health in health_updates:
                if (
                    health.is_global_position_ok
                    and health.is_home_position_ok
                ):
                    break

        print("Waiting for gimbal...")
        gimbal_id = await asyncio.wait_for(
            first_gimbal_id(gimbal),
            timeout=15.0,
        )

        await action.set_takeoff_altitude(TAKEOFF_ALTITUDE_M)

        Path("images").mkdir(exist_ok=True)
        camera = Video()

        armed = False
        controlling_gimbal = False
        offboard_started = False

        # Begin as P-only controllers.
        #
        # Input error is normalized to approximately -1 through +1.
        # Output is velocity in meters per second.
        forward_pid = PID(
            kp=0.7,
            ki=0.0,
            kd=0.0,
            output_limit=MAX_HORIZONTAL_SPEED_M_S,
        )

        right_pid = PID(
            kp=0.7,
            ki=0.0,
            kd=0.0,
            output_limit=MAX_HORIZONTAL_SPEED_M_S,
        )

        try:
            print("Arming...")
            await action.arm()
            armed = True

            print("Taking off...")
            await action.takeoff()
            await asyncio.sleep(10)

            print("Pointing camera downward...")
            await gimbal.take_control(
                gimbal_id,
                ControlMode.PRIMARY,
            )
            controlling_gimbal = True

            await gimbal.set_angles(
                gimbal_id=gimbal_id,
                roll_deg=0.0,
                pitch_deg=-90.0,
                yaw_deg=0.0,
                gimbal_mode=GimbalMode.YAW_LOCK,
                send_mode=SendMode.ONCE,
            )

            await asyncio.sleep(3)

            # PX4 requires an initial setpoint before entering Offboard.
            await send_zero_velocity(offboard)
            await offboard.start()
            offboard_started = True

            print("Offboard mode started")
            print("Searching for red circle...")

            control_start = time.monotonic()
            previous_time = control_start
            centered_since = None
            last_debug_save = 0.0

            while True:
                now = time.monotonic()
                dt = now - previous_time
                previous_time = now

                if now - control_start > MAX_CONTROL_TIME_S:
                    print("Control timeout reached; landing")
                    break

                frame = camera.frame()

                if frame is None:
                    await send_zero_velocity(offboard)
                    await asyncio.sleep(0.1)
                    continue

                # Copy so drawing does not modify camera.py's shared frame.
                frame = frame.copy()
                target = find_red_object(frame)

                height, width = frame.shape[:2]
                image_center_x = width / 2.0
                image_center_y = height / 2.0

                if target is None:
                    print("Red circle not visible; holding position")

                    forward_pid.reset()
                    right_pid.reset()
                    centered_since = None

                    await send_zero_velocity(offboard)
                    await asyncio.sleep(0.1)
                    continue

                target_x, target_y = target

                error_x_px = target_x - image_center_x
                error_y_px = target_y - image_center_y

                # Normalization makes controller behavior less dependent
                # on camera resolution.
                normalized_x = error_x_px / image_center_x
                normalized_y = error_y_px / image_center_y

                forward_error = (
                    IMAGE_X_TO_FORWARD_SIGN * normalized_x
                )

                right_error = (
                    IMAGE_Y_TO_RIGHT_SIGN * normalized_y
                )

                right_velocity = right_pid.update(right_error, dt)
                forward_velocity = forward_pid.update(
                    forward_error,
                    dt,
                )

                distance_px = (
                    error_x_px**2 + error_y_px**2
                ) ** 0.5

                if distance_px <= CENTER_TOLERANCE_PX:
                    await send_zero_velocity(offboard)

                    if centered_since is None:
                        centered_since = now
                        print("Circle centered; confirming...")

                    centered_duration = now - centered_since

                    if centered_duration >= CENTERED_TIME_S:
                        print("Circle remained centered; landing")
                        break
                else:
                    centered_since = None

                    await offboard.set_velocity_body(
                        VelocityBodyYawspeed(
                            forward_m_s=forward_velocity,
                            right_m_s=right_velocity,
                            down_m_s=0.0,
                            yawspeed_deg_s=0.0,
                        )
                    )

                print(
                    f"target=({target_x}, {target_y}) "
                    f"error=({error_x_px:.0f}, {error_y_px:.0f}) "
                    f"velocity=({forward_velocity:.2f} forward, "
                    f"{right_velocity:.2f} right)"
                )

                # Save one updated debugging image per second.
                if now - last_debug_save >= 1.0:
                    last_debug_save = now

                    cv2.drawMarker(
                        frame,
                        (int(image_center_x), int(image_center_y)),
                        (255, 0, 0),
                        cv2.MARKER_CROSS,
                        25,
                        2,
                    )

                    cv2.circle(
                        frame,
                        (target_x, target_y),
                        8,
                        (0, 255, 0),
                        3,
                    )

                    cv2.line(
                        frame,
                        (int(image_center_x), int(image_center_y)),
                        (target_x, target_y),
                        (0, 255, 255),
                        2,
                    )

                    cv2.imwrite(
                        "images/task1_live.jpg",
                        frame,
                    )

                # Approximately 10 control updates per second.
                await asyncio.sleep(0.1)

        finally:
            if offboard_started:
                try:
                    await send_zero_velocity(offboard)
                    await offboard.stop()
                    print("Offboard mode stopped")
                except OffboardError as error:
                    print(f"Could not stop Offboard mode: {error}")

            if armed:
                print("Landing...")
                await action.land()
                await asyncio.sleep(8)

            if controlling_gimbal:
                try:
                    await gimbal.release_control(gimbal_id)
                except Exception as error:
                    print(f"Could not release gimbal: {error}")

            camera.stop()


if __name__ == "__main__":
    asyncio.run(run())