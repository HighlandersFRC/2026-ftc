package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Shooter extends Subsystem {
    DcMotor shooterMotor;
    private ShooterState wantedState = ShooterState.IDLE;
    private ShooterState systemState = ShooterState.IDLE;
    private Drive drive;
    public Shooter(String name, Drive drive) {
        super(name);
        this.drive = drive;
    }

    public void setWantedState(ShooterState state) {
        wantedState = state;
    }

    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Shooter.SHOOTER_NAME);
    }


    public enum ShooterState {
        DEFAULT,
        IDLE,
        SHOOT,
        MANUAL_SHOOT
    }

    public boolean isAtTargetRPM() {
        return true;
    }


    @Override
    public void periodic() {
        systemState = wantedState;

        switch (systemState) {
            case DEFAULT:
                shooterMotor.setPower(0);
                break;
            case IDLE:
                break;
            case SHOOT:
                shooterMotor.setPower(1);
                break;
            case MANUAL_SHOOT:
                shooterMotor.setPower(1);
                break;
        }
    }
}
