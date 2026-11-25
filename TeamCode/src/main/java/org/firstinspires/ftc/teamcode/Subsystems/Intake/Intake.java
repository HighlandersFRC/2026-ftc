package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Superstructure;

public class Intake extends Subsystem {
    DcMotor intakeMotor;
    private IntakeStates wantedState = IntakeStates.IDLE;
    private IntakeStates systemState = IntakeStates.IDLE;

    public Intake (String name) {
        super(name);
    }
    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Intake.INTAKE_NAME);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public enum IntakeStates {
        DEFAULT,
        IDLE,
        INTAKE,
        OUTTAKE
    }

    public void setWantedState(IntakeStates state) {
        wantedState = state;
    }



    @Override
    public void periodic() {
        systemState = wantedState;

        switch (systemState) {
            case IDLE:
            case DEFAULT:
                intakeMotor.setPower(Constants.Intake.DEFAULT_SPEED);
                break;
            case INTAKE:
                intakeMotor.setPower(Constants.Intake.INTAKE_SPEED);
                break;
            case OUTTAKE:
                intakeMotor.setPower(Constants.Intake.OUTTAKE_SPEED);
                break;
        }
    }
}
