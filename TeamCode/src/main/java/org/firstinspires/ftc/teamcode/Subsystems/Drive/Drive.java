package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.LimelightHelpers;
import org.firstinspires.ftc.teamcode.Tools.SubsystemBase;
import org.firstinspires.ftc.teamcode.Tools.Vector;

public class Drive extends SubsystemBase {
    Peripherals peripherals;
    LimelightHelpers limelight;
    DriveIO io;

    private DriveState wantedState = DriveState.DEFAULT;
    private DriveState systemState = DriveState.DEFAULT;


    public Drive(String name, DriveIO driveIO, Peripherals peripherals, LimelightHelpers limelight) {
        super(name);
        this.peripherals = peripherals;
        this.limelight = limelight;
        io = driveIO;
    }

    public void init(HardwareMap hardwareMap) {
        peripherals.init(hardwareMap);
        //limelight.init(hardwareMap);
        io.init(hardwareMap);
    }

    public void followTrajectory(Vector vector, double turnValue, double yawRadians) {
        io.setMotors(vector, turnValue, yawRadians);
    }

    public void stopMotors() {
        io.setMotors(new Vector(0, 0), 0, 0);
    }
    public void teleopFieldCentric(double oiRX, double oiLX, double oiLY, boolean isRed) {
        Vector translation = new Vector(
                oiLX * Constants.Physical.TOP_SPEED,
                -oiLY * Constants.Physical.TOP_SPEED);
        double rotation = oiRX * Constants.Physical.MAX_ANGULAR_SPEED;

        if (systemState == DriveState.ALIGNING) {
            autoAlign(isRed);
        }

        io.setMotors(translation, rotation, Math.toRadians(peripherals.getYaw()));
    }

    private void autoTarget(Pose2D targetPose) {
        Pose2D currentPose = getPose();

        double xPower = Constants.Drive.DrivePID.xController.calculate(currentPose.getX(DistanceUnit.METER), targetPose.getX(DistanceUnit.METER));
        double yPower = Constants.Drive.DrivePID.yController.calculate(currentPose.getY(DistanceUnit.METER), targetPose.getY(DistanceUnit.METER));
        double headingPower = Constants.Drive.DrivePID.headingController.calculate(currentPose.getHeading(AngleUnit.RADIANS), targetPose.getHeading(AngleUnit.RADIANS));

        Vector translation = new Vector(xPower, yPower);

        io.setMotors(translation, headingPower, currentPose.getHeading(AngleUnit.RADIANS));
    }

    public void autoAlign(boolean isRed) {
        Pose2D currentPose = getPose();


        if (Constants.Field.TARGET_TOP_HIVE(currentPose)) {
            autoTarget(isRed ? Constants.Field.RED_TOP_HIVE : Constants.Field.BLUE_TOP_HIVE);
        } else {
            autoTarget(isRed ? Constants.Field.RED_BOTTOM_HIVE : Constants.Field.BLUE_BOTTOM_HIVE);
        }
    }

    public boolean validateAutoAlign(boolean isRed) {
        Pose2D currentPose = getPose();
        Pose2D targetPose;

        if (Constants.Field.TARGET_TOP_HIVE(currentPose)) {
            targetPose = isRed ? Constants.Field.RED_TOP_HIVE : Constants.Field.BLUE_TOP_HIVE;
        } else {
            targetPose = isRed ? Constants.Field.RED_BOTTOM_HIVE : Constants.Field.BLUE_BOTTOM_HIVE;
        }

        return Constants.validatePoseWithinTolerance(targetPose, currentPose, Constants.Drive.DISTANCE_TOLERANCE, Constants.Drive.ANGLE_TOLERANCE);
    }

    private void updateOdometry() {
        peripherals.updateMouse();

//        Pose2D cameraPose = limelight.getPose();
//
//        if (cameraPose != null) {
//            peripherals.fuseCameraOdometry(cameraPose);
//        }
    }

    public void resetOdometry(Pose2D newPose) {
        peripherals.resetOdometry(newPose);
    }

    public Pose2D getPose() {
        return peripherals.getPose();
    }

    public void setWantedState(DriveState state) {
        wantedState = state;
    }

    @Override
    public void periodic() {
        updateOdometry();
        systemState = wantedState;
    }
}
