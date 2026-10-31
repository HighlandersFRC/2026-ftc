package org.firstinspires.ftc.teamcode.Subsystems.Indexer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Indexer extends Subsystem {
    DcMotor indexerMotor;
    private IndexerStates wantedState = IndexerStates.IDLE;
    private IndexerStates systemState = IndexerStates.IDLE;

    public Indexer (String name) {
        super(name);
    }
    public void init(HardwareMap hardwareMap) {
        indexerMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Indexer.INDEXER_NAME);
        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public enum IndexerStates {
        DEFAULT,
        IDLE,
        INDEX
    }

    public void setWantedState(IndexerStates state) {
        wantedState = state;
    }



    @Override
    public void periodic() {
        systemState = wantedState;

        switch (systemState) {
            case DEFAULT:
                indexerMotor.setPower(0);
                break;
            case IDLE:
                break;
            case INDEX:
                indexerMotor.setPower(1);
                break;

        }
    }
}
