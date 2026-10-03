package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

    public class TestBench {
        private DcMotor motor;
        private double ticksPerRev;
        private DigitalChannel touchSensor;

        public void init(HardwareMap hwMap) {
            touchSensor = hwMap.get(DigitalChannel.class, "touch-sensor");
            touchSensor.setMode(DigitalChannel.Mode.INPUT);
            motor = hwMap.get(DcMotor.class, "motor");
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ticksPerRev = motor.getMotorType().getTicksPerRev();
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public boolean isTouchSensorPressed() {
            return !touchSensor.getState();
        }

        public boolean isTouchSensorReleased() {
            return touchSensor.getState();

        }

        public void setMotorSpeed(double speed) {
            motor.setPower(speed);
        }

        public double getMotorRevs() {
            return motor.getCurrentPosition() / ticksPerRev;

        }
        public void setMotorZeroPowerBehavior (DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);

        }
    }





