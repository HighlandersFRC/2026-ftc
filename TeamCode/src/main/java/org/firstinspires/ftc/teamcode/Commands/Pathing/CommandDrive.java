package org.firstinspires.ftc.teamcode.Commands.Pathing;


import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemBase;
import org.firstinspires.ftc.teamcode.Tools.Vector;

public class CommandDrive implements Command {
    Drive drive;
    Peripherals peripherals;
    double x;
    double y;
    double turnValue;

    public CommandDrive(Drive drive, Peripherals peripherals, double x, double y, double turnValue) {
        this.drive = drive;
        this.x = x;
        this.y = y;
        this.turnValue = turnValue;
    }
    @Override
    public void start() {
        drive.followTrajectory(new Vector(x, y), turnValue, Math.toRadians(peripherals.getYaw()));
    }

    @Override
    public void execute() {
        drive.getPose();
    }

    @Override
    public void end() {
        drive.stopMotors();
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(peripherals.getX() - x) <= Constants.Drive.DISTANCE_TOLERANCE &&
            Math.abs(peripherals.getY() - y) <= Constants.Drive.DISTANCE_TOLERANCE &&
            Math.abs(peripherals.getYaw() - turnValue) <= Constants.Drive.ANGLE_TOLERANCE
        ) {
            return true;
        }
        return false;
    }

    @Override
    public SubsystemBase getRequiredSubsystem() {
        return drive;
    }
}
