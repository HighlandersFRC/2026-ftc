package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Limelight extends Subsystem {
    Limelight3A limelightTagDetection;
    Limelight3A limelightObjectDetection;
    public Limelight(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        limelightTagDetection = hardwareMap.get(Limelight3A.class, Constants.InitInfo.Shooter.LIMELIGHT_NAME_TAG_DETECTION);
        limelightTagDetection.start();

        limelightObjectDetection = hardwareMap.get(Limelight3A.class, Constants.InitInfo.Drive.LIMELIGHT_NAME_OBJECT_DETECTION);
        limelightObjectDetection.start();
        limelightObjectDetection.pipelineSwitch(Constants.Drive.LIMELIGHT_OBJECT_DETECTION_PIPELINE);
    }


    public LLResult getResult(Limelight3A limelight) {
        if (limelight == null) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        return result;
    }



    // Mega tag 1
    public Pose2D getPose() {
        LLResult result = getResult(limelightTagDetection);

        if (result == null || result.getBotpose() == null) {
            return null;
        }

        return new Pose2D(
                DistanceUnit.METER,
                result.getBotpose().getPosition().x,
                result.getBotpose().getPosition().y,
                AngleUnit.DEGREES,
                result.getBotpose().getOrientation().getYaw()
        );
    }

    public double getTy() {
        return getResult(limelightTagDetection).getTy();
    }

    public double getTx() {
        return getResult(limelightTagDetection).getTx();
    }

}
