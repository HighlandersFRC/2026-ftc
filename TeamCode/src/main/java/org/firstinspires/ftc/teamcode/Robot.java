package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveIO;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeIO;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterIO;
import org.firstinspires.ftc.teamcode.Subsystems.Superstate;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

@TeleOp
public class Robot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Peripherals peripherals = new Peripherals("Peripherals");
        LimelightHelpers limelight = new LimelightHelpers("Limelight");

        DriveIO driveIO = new DriveIO("Drive");
        Drive drive = new Drive("drive", driveIO, peripherals, limelight);

        IntakeIO intakeIO = new IntakeIO("Intake");
        Intake intake = new Intake("intake", intakeIO);

        ShooterIO shooterIO = new ShooterIO("Shooter");
        Shooter shooter = new Shooter("shooter", shooterIO);
        Superstructure superstructure = new Superstructure("System", false, intake, shooter, drive);
        superstructure.init(hardwareMap);


        waitForStart();

        while(opModeIsActive()) {
            superstructure.periodic();
            // Shooter
            if (gamepad1.right_trigger > 0) {
                drive.autoAlign(false); // False for blue true for red
                superstructure.setWantedState(Superstate.SHOOT);
            } else {
                superstructure.setWantedState(Superstate.DEFAULT);
            }

            if (gamepad1.right_bumper) superstructure.setWantedState(Superstate.MANUAL_SHOOT);
            // Intake
            if (gamepad1.left_trigger > 0) {
                superstructure.setWantedState(Superstate.INTAKE);
            } else {
                superstructure.setWantedState(Superstate.DEFAULT);
            }

            // Drive
            drive.teleopFieldCentric(gamepad1.right_stick_x, gamepad1.left_stick_x, gamepad1.left_stick_y, true);

            if (gamepad1.options) peripherals.configureMouseSensor();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Pose x", peripherals.getPose().getX(DistanceUnit.METER));
            packet.put("Pose y", peripherals.getPose().getY(DistanceUnit.METER));
            packet.put("Pose heading", peripherals.getPose().getHeading(AngleUnit.RADIANS));
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }
    }
}
