package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants.InitInfo.Drive;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

@Autonomous(name = "Cosmos Timed Path", group = "Pathing")
public class TestPathing extends LinearOpMode {
    private static final int FRONT_LEFT = 0;
    private static final int BACK_LEFT = 1;
    private static final int BACK_RIGHT = 2;
    private static final int FRONT_RIGHT = 3;
    private static final double TELEMETRY_INTERVAL_SECONDS = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor[] motors = new DcMotor[4];
        try {
            motors[FRONT_LEFT] = hardwareMap.get(DcMotor.class, Drive.FRONT_LEFT_NAME);
            motors[BACK_LEFT] = hardwareMap.get(DcMotor.class, Drive.BACK_LEFT_NAME);
            motors[BACK_RIGHT] = hardwareMap.get(DcMotor.class, Drive.BACK_RIGHT_NAME);
            motors[FRONT_RIGHT] = hardwareMap.get(DcMotor.class, Drive.FRONT_RIGHT_NAME);
            for (DcMotor motor : motors) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setPower(0);
            }

            List<String> pathFiles = listTrajectoryAssets();
            if (pathFiles.isEmpty()) {
                telemetry.addLine("No Cosmos .traj assets found in paths/.");
                telemetry.update();
                return;
            }

            int selectedIndex = defaultPathIndex(pathFiles);
            boolean leftWasPressed = false;
            boolean rightWasPressed = false;
            telemetry.setMsTransmissionInterval(100);
            while (opModeInInit()) {
                if (gamepad1.dpad_left && !leftWasPressed) {
                    selectedIndex = (selectedIndex + pathFiles.size() - 1) % pathFiles.size();
                }
                if (gamepad1.dpad_right && !rightWasPressed) {
                    selectedIndex = (selectedIndex + 1) % pathFiles.size();
                }
                leftWasPressed = gamepad1.dpad_left;
                rightWasPressed = gamepad1.dpad_right;

                telemetry.addData("Selected path", pathFiles.get(selectedIndex));
                telemetry.addLine("D-pad left/right selects. Press START to run the timed path.");
                telemetry.addLine("Open-loop wheel power; no odometry or mouse sensor is used.");
                telemetry.update();
                sleep(20);
            }
            if (isStopRequested()) return;

            String pathName = pathFiles.get(selectedIndex);
            MecanumTrajectory trajectory;
            try (InputStream input = hardwareMap.appContext.getAssets()
                    .open("paths/" + pathName)) {
                trajectory = MecanumTrajectory.load(input);
            } catch (Exception e) {
                telemetry.addData("Could not load " + pathName, e.getMessage());
                telemetry.update();
                return;
            }

            telemetry.addData("Running", pathName);
            telemetry.addData("Duration", "%.2f s", trajectory.duration());
            telemetry.update();

            long startNanos = System.nanoTime();
            long nextTelemetryNanos = startNanos;
            while (opModeIsActive()) {
                double elapsedSeconds = (System.nanoTime() - startNanos) / 1e9;
                if (elapsedSeconds >= trajectory.duration()) break;

                MecanumTrajectory.State target = trajectory.sample(elapsedSeconds);
                double[] wheelSpeeds = trajectory.wheelSpeeds(target);
                double[] powers = normalizePowers(
                        wheelSpeeds,
                        trajectory.maxWheelSpeedMetersPerSecond());
                for (int i = 0; i < motors.length; i++) {
                    motors[i].setPower(powers[i]);
                }

                long now = System.nanoTime();
                if (now >= nextTelemetryNanos) {
                    nextTelemetryNanos = now + (long) (TELEMETRY_INTERVAL_SECONDS * 1e9);
                    telemetry.addData("Path", pathName);
                    telemetry.addData("Time", "%.2f / %.2f s", elapsedSeconds, trajectory.duration());
                    telemetry.addData("Wheel power FL BL BR FR", "%.2f %.2f %.2f %.2f",
                            powers[FRONT_LEFT], powers[BACK_LEFT], powers[BACK_RIGHT], powers[FRONT_RIGHT]);
                    telemetry.update();
                }
                idle();
            }
        } finally {
            for (DcMotor motor : motors) {
                if (motor != null) motor.setPower(0);
            }
        }
    }

    private List<String> listTrajectoryAssets() {
        List<String> paths = new ArrayList<>();
        try {
            String[] assets = hardwareMap.appContext.getAssets().list("paths");
            if (assets != null) {
                for (String name : assets) {
                    if (name.toLowerCase(Locale.ROOT).endsWith(".traj")) paths.add(name);
                }
            }
        } catch (Exception e) {
            telemetry.addData("Could not list paths", e.getMessage());
        }
        Collections.sort(paths);
        return paths;
    }

    private int defaultPathIndex(List<String> paths) {
        for (int i = 0; i < paths.size(); i++) {
            if (paths.get(i).equalsIgnoreCase("MecanumSmoke.traj")) return i;
        }
        return 0;
    }

    private double[] normalizePowers(double[] wheelSpeeds, double maxWheelSpeed) {
        double[] powers = new double[wheelSpeeds.length];
        double largestMagnitude = 1.0;
        for (int i = 0; i < wheelSpeeds.length; i++) {
            powers[i] = wheelSpeeds[i] / maxWheelSpeed;
            largestMagnitude = Math.max(largestMagnitude, Math.abs(powers[i]));
        }
        for (int i = 0; i < powers.length; i++) powers[i] /= largestMagnitude;
        return powers;
    }
}
