package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure.Superstates;

@TeleOp
public class TestDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Peripherals peripherals = new Peripherals("Peripherals");
        Limelight limelight = new Limelight("Limelight");
        Drive drive = new Drive("Drive", peripherals, limelight);
        drive.init(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {


            // Drive
            drive.teleopFieldCentric(gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_stick_y);

            if (gamepad1.options) peripherals.configureMouseSensor();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Pose x", peripherals.getPose().getX(DistanceUnit.METER));
            packet.put("Pose y", peripherals.getPose().getY(DistanceUnit.METER));
            packet.put("Pose heading", peripherals.getPose().getHeading(AngleUnit.RADIANS));

            packet.put("RX", gamepad1.right_stick_x);
            packet.put("LX", gamepad1.left_stick_x);
            packet.put("LY", gamepad1.left_stick_y);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }
    }
}
