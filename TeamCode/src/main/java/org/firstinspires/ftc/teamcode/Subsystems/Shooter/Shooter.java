package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import static org.firstinspires.ftc.teamcode.Constants.Physical.Ratios.SHOOTER_GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.Constants.Physical.Ratios.SHOOTER_TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.Constants.Physical.Shooter.SHOOTER_RPM;
import static org.firstinspires.ftc.teamcode.Constants.Physical.Shooter.VELOCITY_PID;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Shooter extends Subsystem {
    DcMotorEx shooterMotor;
    private ShooterState wantedState = ShooterState.IDLE;
    private ShooterState systemState = ShooterState.IDLE;
    private Drive drive;
    private double RPM;
    public Shooter(String name, Drive drive) {
        super(name);
        this.drive = drive;
    }

    public void setWantedState(ShooterState state) {
        wantedState = state;
    }

    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, Constants.InitInfo.Shooter.SHOOTER_NAME);
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

    public double getRPM() {
        double shooterRPS = shooterMotor.getVelocity()/SHOOTER_TICKS_PER_REV;
        return 60*shooterRPS/SHOOTER_GEAR_RATIO;
    }

    private void setRPM(double RPM) {
        this.RPM = RPM;
        VELOCITY_PID.setSetPoint(RPM);
    }

    private void updateShooter() {
        VELOCITY_PID.updatePID(getRPM());
        shooterMotor.setPower(VELOCITY_PID.getResult());
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
                //will be replaced with auto RPM later
                break;
            case MANUAL_SHOOT:
                setRPM(SHOOTER_RPM);
                updateShooter();
                break;
        }
    }
}
