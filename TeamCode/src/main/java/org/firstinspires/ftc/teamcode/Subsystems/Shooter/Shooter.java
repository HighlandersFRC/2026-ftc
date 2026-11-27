package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Tools.SubsystemBase;

public class Shooter extends SubsystemBase {

    ShooterIO io;
    private ShooterState wantedState = ShooterState.IDLE;
    private ShooterState systemState = ShooterState.IDLE;
    private double target_position = 0.0;
    public Shooter(String name, ShooterIO shooterIO) {
        super(name);
        io = shooterIO;
    }

    public void setWantedState(ShooterState state) {
        wantedState = state;
    }

    public void init(HardwareMap hardwareMap) {
     io.init(hardwareMap);
    }
    public boolean isAtTargetPosition() {
        double currentPos = io.getCurrentPosition();

        return Constants.withinTolerance(currentPos, Constants.Catapult.CATAPULT_MOTOR_UP, Constants.Catapult.CATAPULT_TOLERANCE);
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

        io.runCatapultMotor(target_position);
    }
}
