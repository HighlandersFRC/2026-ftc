package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Disabled

@TeleOp

public class GamePadPractice extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
       double speedForward = -gamepad1.left_stick_y / 2.0;
       double diff_lXrX = gamepad1.left_stick_x - gamepad1.right_stick_x;
       double sumTriggers = gamepad1.left_trigger + gamepad1.right_trigger;



        telemetry.addData("left X", gamepad1.left_stick_x);
        telemetry.addData("left Y", speedForward);
        telemetry.addData("right x", gamepad1.left_stick_x);
        telemetry.addData("right Y", gamepad1.left_stick_y);
        telemetry.addData("A", gamepad1.a);
        telemetry.addData("B",gamepad1.b);
        telemetry.addData("X and Y difference",diff_lXrX);
        telemetry.addData("Sum of triggers", sumTriggers);
    }
}
