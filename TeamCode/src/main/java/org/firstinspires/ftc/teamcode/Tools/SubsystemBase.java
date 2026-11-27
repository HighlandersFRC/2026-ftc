package org.firstinspires.ftc.teamcode.Tools;

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