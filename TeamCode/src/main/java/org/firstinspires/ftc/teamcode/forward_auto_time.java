package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class forward_auto_time extends LinearOpMode {
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    double time_seconds;

    @Override
    public void runOpMode() throws InterruptedException {
        frontRightMotor = hardwareMap.dcMotor.get("right_front");
        frontLeftMotor = hardwareMap.dcMotor.get("left_front");
        backLeftMotor = hardwareMap.dcMotor.get("left_back");
        backRightMotor = hardwareMap.dcMotor.get("right_back");

        DcMotor[] motors = {
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor
        };
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        waitForStart();
        while (opModeIsActive()) {
            frontLeftMotor.setPower(0.4);
            frontRightMotor.setPower(-0.4);
            backLeftMotor.setPower(0.4);
            backRightMotor.setPower(0.4);

            sleep(800);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

            break;

        }
    }

}
