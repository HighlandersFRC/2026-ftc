package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Pathing.Follower;
import org.firstinspires.ftc.teamcode.Commands.Pathing.PurePursuit.PurepursuitFollower;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Peripherals;

@Autonomous
public class TestPathing extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Peripherals peripherals = new Peripherals("Peripherals");
        Limelight limelight = new Limelight("ll");
        Drive drive = new Drive("drive");
        CommandScheduler scheduler = new CommandScheduler();
        PurepursuitFollower path = new PurepursuitFollower("OneMeterX.polarpath", drive);

        try {
            drive.init(hardwareMap);
            boolean loaded = path.prepare();
            telemetry.setMsTransmissionInterval(100);
            while (opModeInInit()) {
                drive.periodic();
                Pose2D raw = drive.getPose();
                telemetry.addData("Path", path.getStatus());
                telemetry.addData("Raw sensor X (m)", raw.getX(DistanceUnit.METER));
                telemetry.addData("Raw sensor Y (m)", raw.getY(DistanceUnit.METER));
                telemetry.addData("Raw heading (deg)", raw.getHeading(AngleUnit.DEGREES));
                telemetry.addLine("Check pose by pushing/turning the robot before START.");
                telemetry.update();
                sleep(10);
            }
            if (isStopRequested() || !loaded) return;

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
            path.end();
        }
    }
}
