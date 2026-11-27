package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemBase;

public class IntakeIO extends SubsystemBase {
    DcMotor intakeMotor;
    public IntakeIO(String name) {
        super(name);
    }

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get(Constants.InitInfo.Intake.INTAKE_NAME);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPowerPercent(double percent) {
        intakeMotor.setPower(percent / 100);
    }

    @Override
    public void periodic() {super.periodic();}
}
