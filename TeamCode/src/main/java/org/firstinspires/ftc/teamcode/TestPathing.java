package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Pathing.PurePursuit.PurepursuitFollower;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Peripherals;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

@Autonomous(name = "Cosmos Path Picker", group = "Pathing")
public class TestPathing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Peripherals peripherals = new Peripherals("Peripherals");
        Limelight limelight = new Limelight("ll");
        Drive drive = new Drive("drive", peripherals, limelight);
        CommandScheduler scheduler = new CommandScheduler();
        PurepursuitFollower path = null;
        boolean driveInitialized = false;

        try {
            drive.init(hardwareMap);
            driveInitialized = true;
            List<String> pathFiles = listPathAssets();
            if (pathFiles.isEmpty()) {
                telemetry.addLine("No .traj or .polarpath assets found in paths/.");
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

                drive.periodic();
                Pose2D raw = drive.getPose();
                telemetry.addData("Selected path", pathFiles.get(selectedIndex));
                telemetry.addLine("D-pad left/right selects a path. Press START to run it.");
                telemetry.addData("Raw sensor X (m)", raw.getX(DistanceUnit.METER));
                telemetry.addData("Raw sensor Y (m)", raw.getY(DistanceUnit.METER));
                telemetry.addData("Raw heading (deg)", raw.getHeading(AngleUnit.DEGREES));
                telemetry.update();
                sleep(20);
            }
            if (isStopRequested()) return;

            path = new PurepursuitFollower(pathFiles.get(selectedIndex), drive);
            if (!path.prepare()) {
                telemetry.addData("Path load failed", path.getStatus());
                telemetry.update();
                return;
            }

            scheduler.schedule(path);
            long nextTelemetry = 0;
            while (opModeIsActive()) {
                drive.periodic();
                scheduler.run();

                long now = System.nanoTime();
                if (now >= nextTelemetry) {
                    nextTelemetry = now + 100_000_000L;
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("Follower/status", path.getStatus());
                    packet.put("Follower/success", path.isSuccessful());
                    packet.put("Follower/current X (m)", path.getCurrentX());
                    packet.put("Follower/current Y (m)", path.getCurrentY());
                    packet.put("Follower/target X (m)", path.getTargetX());
                    packet.put("Follower/target Y (m)", path.getTargetY());
                    packet.put("Follower/position error (m)", path.getPositionError());
                    packet.put("Follower/heading error (deg)", Math.toDegrees(path.getHeadingError()));
                    packet.put("Follower/robot X power", path.getRobotXPower());
                    packet.put("Follower/robot Y power", path.getRobotYPower());
                    packet.put("Follower/CCW turn power", path.getTurnPower());

                    packet.put("Pose x", peripherals.getPose().getX(DistanceUnit.METER));
                    packet.put("Pose y", peripherals.getPose().getY(DistanceUnit.METER));
                    packet.put("Pose heading", peripherals.getPose().getHeading(AngleUnit.RADIANS));
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);

                    telemetry.addData("Path", path.getStatus());
                    telemetry.addData("Successful", path.isSuccessful());
                    telemetry.addData("Position error", "%.3f m", path.getPositionError());
                    telemetry.addData("Heading error", "%.1f deg", Math.toDegrees(path.getHeadingError()));
                    telemetry.update();
                }
                if (path.isFinished()) break;
                sleep(10);
            }
        } finally {
            if (path != null) path.end();
            else if (driveInitialized) drive.stopMotors();
        }
    }

    private List<String> listPathAssets() {
        List<String> paths = new ArrayList<>();
        try {
            String[] assets = hardwareMap.appContext.getAssets().list("paths");
            if (assets != null) {
                for (String name : assets) {
                    String lower = name.toLowerCase(Locale.ROOT);
                    if (lower.endsWith(".traj") || lower.endsWith(".polarpath")
                            || lower.endsWith(".polarpaths")) {
                        paths.add(name);
                    }
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
}
