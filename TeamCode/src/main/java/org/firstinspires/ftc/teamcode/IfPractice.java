package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IfPractice extends OpMode {


    @Override
    public void init() {

    }

    @Override
    public void loop() {
        double motorSpeed = gamepad1.left_stick_y;

        if (!gamepad1.a) {
            motorSpeed *= 0.5;
        }
        telemetry.addData("Left Stick Value", motorSpeed);

        /*
        if (leftY <0.1 && leftY > -0.1) {
            telemetry.addData("Left Stick", "In Dead Zone");
        }
        */

        /*
         boolean aButton = gamepad1.a;

        if (aButton) {
            telemetry.addData("A Button", "Pressed!");

        }
        else {
            telemetry.addData("A Button", "Not Pressed!");
        }
        telemetry.addData("A Button State", aButton);

        double leftY = gamepad1.left_stick_y;
            if (leftY < 0) {
                telemetry.addData("Left Stick", "Is Negative");
            }
            else if (leftY > 0.5) {
                telemetry.addData("Left Stick", "Greater than 50%");
            }
            else if (leftY > 0) {
            telemetry.addData("Left Stick", "Is Greater than 0");
        }
            else{
                telemetry.addData("Left Stick", "Is Positive");
        */

    }
}
