package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

@TeleOp
public class IndexerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Peripherals peripherals = new Peripherals("peripherals");
        Limelight limelight = new Limelight("limelight");
        Drive drive = new Drive("drive", peripherals, limelight);
        Intake intake = new Intake("intake");
        Shooter shooter = new Shooter("shooter", drive);
        Indexer indexer = new Indexer("indexer");
        Superstructure superstructure = new Superstructure("System", intake, shooter, indexer, drive);
        superstructure.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            superstructure.periodic();

            superstructure.setWantedState(Superstructure.Superstates.INDEX);
        }
    }
}
