package org.firstinspires.ftc.teamcode.Tools;

public interface Command {

    void start();

    void execute();

    void end();

    boolean isFinished();

    SubsystemBase getRequiredSubsystem();


}