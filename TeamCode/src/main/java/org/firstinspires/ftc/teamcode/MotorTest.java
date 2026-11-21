package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Simple Motor Test", group = "Test")
public class MotorTest extends LinearOpMode {
    private static final String MOTOR_NAME = "left_front";
    private static final double MOTOR_POWER = 0.25;

    @Override
    public void runOpMode() {
        DcMotor motor = hardwareMap.get(DcMotor.class, MOTOR_NAME);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Motor will run at fixed power until stopped.");
        telemetry.addData("Motor", MOTOR_NAME);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(MOTOR_POWER);
            idle();
        }

        motor.setPower(0);
    }
}
