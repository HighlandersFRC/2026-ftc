package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.Vector;

public class Drive extends Subsystem {
    Peripherals peripherals;
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;

    Limelight limelight;
    public Drive(String name, Peripherals peripherals, Limelight limelight) {
        super(name);
        this.peripherals = peripherals;
        this.limelight = limelight;
    }



    public void init(HardwareMap hardwareMap) {
        peripherals.init(hardwareMap);
        //limelight.init(hardwareMap);
        frontRightMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Drive.FRONT_RIGHT_NAME);
        frontLeftMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Drive.FRONT_LEFT_NAME);
        backRightMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Drive.BACK_RIGHT_NAME);
        backLeftMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Drive.BACK_LEFT_NAME);
    }

    public void setMotors(Vector vector, double turnValue, double yawRadians) {
        double cos = Math.cos(yawRadians);
        double sin = Math.sin(yawRadians);

        double x = vector.getI() * cos + vector.getJ() * sin;
        double y = -vector.getI() * sin + vector.getJ() * cos;

        double frontLeft  = -y + x - turnValue;
        double frontRight = y + x - turnValue;
        double backLeft   = y + x + turnValue;
        double backRight  = y - x - turnValue;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(frontLeft),
                        Math.max(
                                Math.abs(frontRight),
                                Math.max(Math.abs(backLeft), Math.abs(backRight))
                        )
                )
        );

        frontLeft  /= max;
        frontRight /= max;
        backLeft   /= max;
        backRight  /= max;

        frontLeftMotor.setPower(frontLeft);
        frontRightMotor.setPower(frontRight);
        backLeftMotor.setPower(backLeft);
        backRightMotor.setPower(backRight);
    }

    public void stopMotors() {
        setMotors(new Vector(0, 0), 0, 0);
    }
    public void teleopFieldCentric(double oiRX, double oiLX, double oiLY) {
        Vector translation = new Vector(
                oiLX * Constants.Physical.TOP_SPEED,
                -oiLY * Constants.Physical.TOP_SPEED);
        double rotation = oiRX * Constants.Physical.MAX_ANGULAR_SPEED;

        setMotors(translation, rotation, Math.toRadians(peripherals.getYaw()));
    }

    private void autoTarget(Pose2D targetPose) {
        Pose2D currentPose = getPose();

        double xPower = Constants.Drive.DrivePID.xController.calculate(currentPose.getX(DistanceUnit.METER), targetPose.getX(DistanceUnit.METER));
        double yPower = Constants.Drive.DrivePID.yController.calculate(currentPose.getY(DistanceUnit.METER), targetPose.getY(DistanceUnit.METER));
        double headingPower = Constants.Drive.DrivePID.headingController.calculate(currentPose.getHeading(AngleUnit.RADIANS), targetPose.getHeading(AngleUnit.RADIANS));

        Vector translation = new Vector(xPower, yPower);

        setMotors(translation, headingPower, currentPose.getHeading(AngleUnit.RADIANS));
    }

    public void autoAlign(boolean isRed) {
        Pose2D currentPose = getPose();


        if (Constants.Field.TARGET_TOP_HIVE(currentPose)) {
            autoTarget(isRed ? Constants.Field.RED_TOP_HIVE : Constants.Field.BLUE_TOP_HIVE);
        } else {
            autoTarget(isRed ? Constants.Field.RED_BOTTOM_HIVE : Constants.Field.BLUE_BOTTOM_HIVE);
        }
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

    @Override
    public void periodic() {
        updateOdometry();
    }
}
