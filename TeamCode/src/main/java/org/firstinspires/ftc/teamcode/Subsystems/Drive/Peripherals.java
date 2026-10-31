package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import static org.firstinspires.ftc.teamcode.Constants.Physical.IMU_NAME;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Peripherals extends Subsystem {
    private double fieldX;
    private double fieldY;
    private double theta;
    SparkFunOTOS mouse;
    IMU imu;

    public Peripherals(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {

        mouse = hardwareMap.get(SparkFunOTOS.class, Constants.InitInfo.Drive.MOUSE_NAME);
        mouse.setLinearUnit(DistanceUnit.METER);
        mouse.setAngularUnit(AngleUnit.DEGREES);
        mouse.setOffset(Constants.Physical.MOUSE_SENSOR_MOUNTED_POSE);
        mouse.setLinearScalar(1.0);
        mouse.setAngularScalar(Constants.Physical.MOUSE_SENSOR_ANGULAR_SCALAR);
        configureMouseSensor();

        imu = hardwareMap.get(IMU.class, IMU_NAME);
        RevHubOrientationOnRobot HubOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }

    public void configureMouseSensor() {
    mouse.calibrateImu();
    mouse.resetTracking();
    }

    public void resetOdometry(Pose2D newPose) {
        mouse.setPosition(new SparkFunOTOS.Pose2D(newPose.getX(DistanceUnit.METER), newPose.getY(DistanceUnit.METER), newPose.getHeading(AngleUnit.DEGREES)));
    }

    public void updateMouse() {
        SparkFunOTOS.Pose2D field = mouse.getPosition();
        fieldX = field.x;
        fieldY = field.y;
        theta = field.h;
    }

    public Pose2D getPose() {
        return new Pose2D(DistanceUnit.METER, fieldX * Constants.Physical.MOUSE_SENSOR_LINEAR_SCALAR, fieldY * Constants.Physical.MOUSE_SENSOR_LINEAR_SCALAR, AngleUnit.DEGREES, theta);
    }

    public void fuseCameraOdometry(Pose2D cameraPose) {
        if (cameraPose == null) return;

        SparkFunOTOS.Pose2D otosPose = mouse.getPosition();

        double otosX = otosPose.x;
        double otosY = otosPose.y;
        double otosHeading = otosPose.h;

        double cameraX = cameraPose.getX(DistanceUnit.METER);
        double cameraY = cameraPose.getY(DistanceUnit.METER);
        double cameraHeading = cameraPose.getHeading(AngleUnit.DEGREES);

        double dx = cameraX - otosX;
        double dy = cameraY - otosY;

        double distanceError = Math.hypot(dx, dy);

        double headingError =
                normalizeAngle(cameraHeading - otosHeading);

        if (distanceError > 0.5) return;
        if (Math.abs(headingError) > 30.0) return;

        final double POSITION_WEIGHT = 0.15;
        final double HEADING_WEIGHT = 0.15;

        double fusedX =
                otosX + dx * POSITION_WEIGHT;

        double fusedY =
                otosY + dy * POSITION_WEIGHT;

        double fusedHeading =
                otosHeading + headingError * HEADING_WEIGHT;

        fusedHeading = normalizeAngle(fusedHeading);

        mouse.setPosition(
                new SparkFunOTOS.Pose2D(
                        fusedX,
                        fusedY,
                        fusedHeading
                )
        );

        fieldX = fusedX;
        fieldY = fusedY;
        theta = fusedHeading;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }

        while (angle < -180.0) {
            angle += 360.0;
        }

        return angle;
    }

    public double getX() {
        return fieldX * Constants.Physical.MOUSE_SENSOR_LINEAR_SCALAR;
    }

    public double getY() {
        return fieldY * Constants.Physical.MOUSE_SENSOR_LINEAR_SCALAR;
    }

    public double getYaw() {
        return theta;
    }

    public double getYawIMU() {
        return imu.getRobotYawPitchRollAngles().getYaw();
    }

}
