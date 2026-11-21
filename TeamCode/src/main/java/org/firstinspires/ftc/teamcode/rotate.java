package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class rotate extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("left_front");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("right_front");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("left_back");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("right_back");

        waitForStart();
        while (opModeIsActive()){

            frontLeftMotor.setPower(0.5);
            frontRightMotor.setPower(-0.5);
            backLeftMotor.setPower(0.5);
            backRightMotor.setPower(0.5);

            sleep(500);

            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            break;
        }
    }
}
