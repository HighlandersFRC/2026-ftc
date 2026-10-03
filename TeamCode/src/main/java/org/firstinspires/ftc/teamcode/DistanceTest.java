package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Mechanisms.TestBenchDistance;
@TeleOp
public class DistanceTest extends OpMode {
    TestBenchDistance bench = new TestBenchDistance();
    double distance;

    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        distance = bench.getDistance();

        if (distance < 10) {

            telemetry.addLine("Too Close!");

        }
    }
}