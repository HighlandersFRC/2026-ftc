package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveState;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterState;

public class Superstructure extends SubsystemBase {
    private final Intake intake;
    private final Shooter shooter;
    private final Drive drive;
    private Superstate wantedState = Superstate.IDLE;
    private Superstate systemState = Superstate.IDLE;

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

    public void setWantedState(Superstate state) {
        wantedState = state;
    }



    @Override
    public void periodic() {
        systemState = wantedState;

        drive.periodic();
        intake.periodic();
        shooter.periodic();
        switch (systemState) {
            case DEFAULT:
                intake.setWantedState(IntakeState.DEFAULT);
                drive.setWantedState(DriveState.DEFAULT);
                break;
            case IDLE:
                intake.setWantedState(IntakeState.IDLE);
                drive.setWantedState(DriveState.DEFAULT);
                break;
            case INTAKE:
                intake.setWantedState(IntakeState.INTAKE);
                drive.setWantedState(DriveState.DEFAULT);
                break;
            case OUTTAKE:
                intake.setWantedState(IntakeState.OUTTAKE);
                drive.setWantedState(DriveState.DEFAULT);
                break;
            case SHOOT:
                shooter.setWantedState(ShooterState.SHOOT);
                drive.setWantedState(DriveState.ALIGNING);
                if (shooter.isAtTargetPosition() && drive.validateAutoAlign(isRed)) {
                    shooter.setWantedState(ShooterState.DEFAULT);
                    drive.setWantedState(DriveState.DEFAULT);
                }
                break;
            case MANUAL_SHOOT:
                shooter.setWantedState(ShooterState.SHOOT);
                drive.setWantedState(DriveState.DEFAULT);
                if (shooter.isAtTargetPosition()) {
                    shooter.setWantedState(ShooterState.DEFAULT);
                }
                break;
        }
    }

}
