# Simple Motor Test

This branch keeps the FTC SDK and replaces the team's custom Java OpModes with one fixed-power motor test.

## Hardware setup

In the Robot Controller configuration, add or select a DC motor with the exact configured name `left_front`.

## Run it

1. Connect the Control Hub to the computer over USB and make sure it appears as an ADB device.
2. In VS Code, choose **Terminal → Run Task… → FTC: Install on connected Robot Controller**.
3. On the Driver Station, select **Simple Motor Test**, press **INIT**, then **START**.

The OpMode runs the `left_front` motor continuously at 25% forward power until the OpMode is stopped. It does not use gamepad input.

## Change the motor or power

Edit `MotorTest.java`:

- `MOTOR_NAME` must match the motor's Robot Controller configuration name.
- `MOTOR_POWER` is a value from `-1.0` to `1.0`; positive runs forward and negative runs in reverse.

Keep the robot secured while testing a drive motor.
