package org.firstinspires.ftc.teamcode.Subsystems;

public abstract class Subsystem {
    private String name;

    public Subsystem(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }
    public void periodic(){}
}