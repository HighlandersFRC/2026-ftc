package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Tools.SubsystemBase;
import org.firstinspires.ftc.teamcode.Tools.Vector;

public class DriveIO extends SubsystemBase {

    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor backLeftMotor;


    public DriveIO(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        frontRightMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Drive.FRONT_RIGHT_NAME);
        frontLeftMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Drive.FRONT_LEFT_NAME);
        backRightMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Drive.BACK_RIGHT_NAME);
        backLeftMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Drive.BACK_LEFT_NAME);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setMotors(Vector vector, double turnValue, double yawRadians) {
        double cos = Math.cos(yawRadians);
        double sin = Math.sin(yawRadians);

        double x = vector.getI() * cos + vector.getJ() * sin;
        double y = -vector.getI() * sin + vector.getJ() * cos;

        double frontLeft  = y + x + turnValue;
        double backLeft   = y - x + turnValue;
        double frontRight = y - x - turnValue;
        double backRight  = y + x - turnValue;

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

    @Override
    public void periodic() {
        super.periodic();
    }
}
