package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;

@TeleOp
public class ShooterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive("Drive");
        Shooter shooter = new Shooter("Shooter", drive);
        shooter.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            shooter.setRPM(2500);

            shooter.periodic();
            shooter.setWantedState(Shooter.ShooterState.SHOOT);

            double shooterRPM = shooter.getRPM();
            telemetry.addData("Shooter RPM", shooterRPM);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Shooter RPM", shooterRPM);
            packet.put("Target RPM", 2500);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
