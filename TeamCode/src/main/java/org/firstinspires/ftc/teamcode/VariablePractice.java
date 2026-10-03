package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled

@TeleOp

public class VariablePractice extends OpMode {
    @Override
    public void init() {
        int teamNumber = 1000;
        double motorSpeed = 1;
        boolean clawClosed = true;
        String teamName = "TEST";
        int motorAngle = 157;

        telemetry.addData("Team number", teamNumber);
        telemetry.addData("Motor speed", motorSpeed);
        telemetry.addData("Claw closed", clawClosed);
        telemetry.addData("Team name", teamName);
        telemetry.addData("Motor angle", motorAngle);
    }

    @Override
    public void loop() {

    }
}
