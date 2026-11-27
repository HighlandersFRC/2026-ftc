package org.firstinspires.ftc.teamcode.Subsystems;

public abstract class SubsystemBase {
    private String name;

    public SubsystemBase(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }
    public void periodic(){}
}