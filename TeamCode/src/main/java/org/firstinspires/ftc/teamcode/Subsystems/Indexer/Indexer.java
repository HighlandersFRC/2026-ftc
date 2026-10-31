package org.firstinspires.ftc.teamcode.Subsystems.Indexer;

import static org.firstinspires.ftc.teamcode.Constants.InitInfo.Indexer.INDEXER_NAME;
import static org.firstinspires.ftc.teamcode.Constants.InitInfo.Indexer.QUEUER_NAME;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Indexer extends Subsystem {
    DcMotor indexerMotor;
    DcMotor queuerMotor;
    private IndexerStates wantedState = IndexerStates.IDLE;
    private IndexerStates systemState = IndexerStates.IDLE;

    public Indexer (String name) {
        super(name);
    }
    public void init(HardwareMap hardwareMap) {
        indexerMotor = hardwareMap.dcMotor.get(INDEXER_NAME);
        queuerMotor = hardwareMap.dcMotor.get(QUEUER_NAME);
        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public enum IndexerStates {
        DEFAULT,
        IDLE,
        INDEX,
        QUEUE
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
            case QUEUE:
                queuerMotor.setPower(1);
                break;

        }
    }
}
