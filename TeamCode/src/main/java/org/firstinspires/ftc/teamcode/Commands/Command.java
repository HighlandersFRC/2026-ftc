package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Subsystems.SubsystemBase;

public interface Command {

    void start();

    void execute();

    void end();

    boolean isFinished();

    SubsystemBase getRequiredSubsystem();


}