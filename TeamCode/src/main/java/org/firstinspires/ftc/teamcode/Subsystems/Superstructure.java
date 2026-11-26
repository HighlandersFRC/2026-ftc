package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;

public class Superstructure extends Subsystem{
    private final Intake intake;
    private final Shooter shooter;
    private final Drive drive;
    private Superstates wantedState = Superstates.IDLE;
    private Superstates systemState = Superstates.IDLE;

    private boolean isRed;

    public Superstructure (String name, boolean isRed, Intake intake, Shooter shooter, Drive drive) {
        super(name);
        this.intake = intake;
        this.shooter = shooter;
        this.drive = drive;
        this.isRed = isRed;
    }

    public void init(HardwareMap hardwareMap) {
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        drive.init(hardwareMap);
    }

    public void setWantedState(Superstates state) {
        wantedState = state;
    }


    public enum Superstates {
        DEFAULT,
        IDLE,
        INTAKE,
        OUTTAKE,
        SHOOT,
        MANUAL_SHOOT
    }

    @Override
    public void periodic() {
        systemState = wantedState;

        drive.periodic();
        intake.periodic();
        shooter.periodic();
        switch (systemState) {
            case DEFAULT:
                intake.setWantedState(Intake.IntakeStates.DEFAULT);
                break;
            case IDLE:
                intake.setWantedState(Intake.IntakeStates.IDLE);
                break;
            case INTAKE:
                intake.setWantedState(Intake.IntakeStates.INTAKE);
                break;
            case OUTTAKE:
                intake.setWantedState(Intake.IntakeStates.OUTTAKE);
                break;
            case SHOOT:
                shooter.setWantedState(Shooter.ShooterState.SHOOT);
                if (shooter.isAtTargetPosition() && drive.validateAutoAlign(isRed)) {
                    shooter.setWantedState(Shooter.ShooterState.DEFAULT);
                }
                break;
            case MANUAL_SHOOT:
                shooter.setWantedState(Shooter.ShooterState.SHOOT);
                if (shooter.isAtTargetPosition()) {
                    shooter.setWantedState(Shooter.ShooterState.DEFAULT);
                }
                break;
        }
    }

}
