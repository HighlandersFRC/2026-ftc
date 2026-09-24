package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Indexer.Indexer;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;

public class Superstructure extends Subsystem{
    private final Intake intake;
    private final Shooter shooter;
    private final Indexer indexer;
    private final Drive drive;
    private Superstates wantedState = Superstates.IDLE;
    private Superstates systemState = Superstates.IDLE;

    public Superstructure (String name, Intake intake, Shooter shooter, Indexer indexer, Drive drive) {
        super(name);
        this.intake = intake;
        this.shooter = shooter;
        this.indexer = indexer;
        this.drive = drive;
    }

    public void init(HardwareMap hardwareMap) {
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
        indexer.init(hardwareMap);
    }

    public void setWantedState(Superstates state) {
        wantedState = state;
    }


    public enum Superstates {
        DEFAULT,
        IDLE,
        INTAKE,
        OUTTAKE,
        SHOOT
    }

    @Override
    public void periodic() {
        systemState = wantedState;

        drive.periodic();
        indexer.periodic();
        intake.periodic();
        shooter.periodic();
        switch (systemState) {
            case DEFAULT:
                intake.setWantedState(Intake.IntakeStates.DEFAULT);
                shooter.setWantedState(Shooter.ShooterState.DEFAULT);
                indexer.setWantedState(Indexer.IndexerStates.DEFAULT);
                break;
            case IDLE:
                intake.setWantedState(Intake.IntakeStates.IDLE);
                shooter.setWantedState(Shooter.ShooterState.IDLE);
                break;
            case INTAKE:
                intake.setWantedState(Intake.IntakeStates.INTAKE);
                break;
            case OUTTAKE:
                intake.setWantedState(Intake.IntakeStates.OUTTAKE);
                break;
            case SHOOT:
                shooter.setWantedState(Shooter.ShooterState.SHOOT);
                if (shooter.isAtTargetRPM()) {
                    indexer.setWantedState(Indexer.IndexerStates.INDEX);
                }
                break;
        }
    }

}
