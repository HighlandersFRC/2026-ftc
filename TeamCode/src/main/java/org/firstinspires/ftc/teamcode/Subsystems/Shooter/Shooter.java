package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Shooter extends Subsystem {
    DcMotor catapultMotor;
    private ShooterState wantedState = ShooterState.IDLE;
    private ShooterState systemState = ShooterState.IDLE;
    private double target_position = 0.0;
    public Shooter(String name) {
        super(name);
    }

    public void setWantedState(ShooterState state) {
        wantedState = state;
    }

    public void init(HardwareMap hardwareMap) {
        catapultMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Shooter.SHOOTER_NAME);
        catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public enum ShooterState {
        DEFAULT,
        IDLE,
        SHOOT,
    }

    public boolean isAtTargetPosition() {
        double currentPos = catapultMotor.getCurrentPosition();

        return Constants.withinTolerance(currentPos, Constants.Catapult.CATAPULT_MOTOR_UP, Constants.Catapult.CATAPULT_TOLERANCE);
    }

    public void runCatapultMotor() {
        double targetPos = target_position; // In future so I can change here quickly
        double currentPos = catapultMotor.getCurrentPosition();
        double result = Constants.Catapult.CATAPULT_TARGET_PID.calculate(currentPos, targetPos);

        catapultMotor.setPower(result);
    }


    @Override
    public void periodic() {
        systemState = wantedState;

        switch (systemState) {
            case IDLE:
            case DEFAULT:
                target_position = Constants.Catapult.CATAPULT_MOTOR_ZERO;
                break;
            case SHOOT:
                target_position = Constants.Catapult.CATAPULT_MOTOR_UP;
                break;
        }

        runCatapultMotor();
    }
}
