package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemBase;

public class Intake extends SubsystemBase {

    private IntakeState wantedState = IntakeState.IDLE;
    private IntakeState systemState = IntakeState.IDLE;
    IntakeIO io;

    public Intake (String name, IntakeIO intakeIO) {
        super(name);
        io = intakeIO;
    }
    public void init(HardwareMap hardwareMap) {
        io.init(hardwareMap);
    }


    public void setWantedState(IntakeState state) {
        wantedState = state;
    }



    @Override
    public void periodic() {
        systemState = wantedState;

        switch (systemState) {
            case IDLE:
            case DEFAULT:
                io.setPowerPercent(Constants.Intake.DEFAULT_SPEED);
                break;
            case INTAKE:
                io.setPowerPercent(Constants.Intake.INTAKE_SPEED);
                break;
            case OUTTAKE:
                io.setPowerPercent(Constants.Intake.OUTTAKE_SPEED);
                break;
        }
    }
}
