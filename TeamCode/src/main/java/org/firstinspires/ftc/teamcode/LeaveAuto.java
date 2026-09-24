package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.Commands.Pathing.CommandDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Peripherals;

@Autonomous
public class LeaveAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Peripherals peripherals = new Peripherals("Peripherals");
        Limelight limelight = new Limelight("ll");
        Drive drive = new Drive("drive", peripherals, limelight);
        CommandScheduler scheduler = new CommandScheduler();

        drive.init(hardwareMap);

        scheduler.schedule(new CommandDrive(drive, peripherals, 1, 0, 0));

        while (opModeIsActive()) {
            peripherals.updateMouse();
            scheduler.run();

            peripherals.periodic();
            drive.periodic();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Pose x", peripherals.getPose().getX(DistanceUnit.METER));
            packet.put("Pose y", peripherals.getPose().getY(DistanceUnit.METER));
            packet.put("Pose heading", peripherals.getPose().getHeading(AngleUnit.RADIANS));
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
