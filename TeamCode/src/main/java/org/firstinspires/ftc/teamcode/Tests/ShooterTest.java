package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Constants.Physical.Shooter.SHOOTER_RPM;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

@TeleOp
public class ShooterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Peripherals peripherals = new Peripherals("peripherals");
        Limelight limelight = new Limelight("limelight");
        Drive drive = new Drive("Drive", peripherals, limelight);
        Shooter shooter = new Shooter("Shooter", drive);
        Superstructure superstructure = new Superstructure("System", null, shooter, null, drive);
        superstructure.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            superstructure.periodic();
            superstructure.setWantedState(Superstructure.Superstates.MANUAL_SHOOT);

            double shooterRPM = shooter.getRPM();
            telemetry.addData("Shooter RPM", shooterRPM);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Shooter RPM", shooterRPM);
            packet.put("Target RPM", SHOOTER_RPM);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
