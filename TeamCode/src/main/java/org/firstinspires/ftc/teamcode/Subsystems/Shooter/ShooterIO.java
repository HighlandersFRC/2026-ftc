package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemBase;

public class ShooterIO extends SubsystemBase {
    DcMotor catapultMotor;
    public ShooterIO(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        catapultMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Shooter.SHOOTER_NAME);
        catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double getCurrentPosition() {
        return catapultMotor.getCurrentPosition();
    }

    public void runCatapultMotor(double targetPos) {
        double currentPos = catapultMotor.getCurrentPosition();
        double result = Constants.Catapult.CATAPULT_TARGET_PID.calculate(currentPos, targetPos);

        catapultMotor.setPower(result);
    }


    @Override
    public void periodic() {super.periodic();}
}
