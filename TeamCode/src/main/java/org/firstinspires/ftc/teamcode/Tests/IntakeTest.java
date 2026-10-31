package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

@TeleOp
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Peripherals peripherals = new Peripherals("peripherals");
        Limelight limelight = new Limelight("limelight");
        Intake intake = new Intake("Intake");
        Drive drive = new Drive("Drive", peripherals, limelight);
        Shooter shooter = new Shooter("Shooter", drive);
        Superstructure superstructure = new Superstructure("System", intake, shooter, null, drive);
        superstructure.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            superstructure.periodic();
            if (gamepad1.left_trigger > 0) superstructure.setWantedState(Superstructure.Superstates.INTAKE);
            else if (gamepad1.left_bumper) superstructure.setWantedState(Superstructure.Superstates.OUTTAKE);
            else superstructure.setWantedState(Superstructure.Superstates.DEFAULT);
        }
    }
}
