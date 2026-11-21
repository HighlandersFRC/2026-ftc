
package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.util.RobotLog;
import java.util.*;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;


public class CommandScheduler {
    private static CommandScheduler instance;
    private final List<Command> scheduledCommands = new ArrayList<>();
    private final Map<Subsystem, Command> activeSubsystemCommands = new HashMap<>();

    public CommandScheduler() {

    }

    public static CommandScheduler getInstance() {
        if (instance == null) {
            instance = new CommandScheduler();
        }
        return instance;
    }



    public void schedule(Command command) {
        Subsystem requiredSubsystem = command.getRequiredSubsystem();

        if (requiredSubsystem != null) {
            Command activeCommand = activeSubsystemCommands.get(requiredSubsystem);

            if (activeCommand != null && activeCommand != command) {
                cancel(activeCommand);
            }

            activeSubsystemCommands.put(requiredSubsystem, command);
        }


        // Schedule and start the new command if not already in the list
        if (!scheduledCommands.contains(command)) {
            scheduledCommands.add(command);
            command.start();
            RobotLog.d("Command Scheduled: " + command.getClass().getSimpleName());
        }
    }
    public boolean isSubsystemBusy(Subsystem subsystem) {
        return activeSubsystemCommands.containsKey(subsystem);
    }

    public void run() {
        List<Command> finishedCommands = new ArrayList<>();

        // Execute scheduled commands and handle completion
        for (Command command : new ArrayList<>(scheduledCommands)) {
            if (command.isFinished()) {
                command.end();
                finishedCommands.add(command);
                RobotLog.d("Command Finished and Ended: " + command.getClass().getSimpleName());

                Subsystem subsystem = command.getRequiredSubsystem();
                if (subsystem != null) {
                    activeSubsystemCommands.remove(subsystem);

                    // Only reschedule default command if no other commands are active for this subsystem
                }
            } else {
                command.execute();
            }
        }

        scheduledCommands.removeAll(finishedCommands);

        // Ensure default commands are scheduled when needed
    }


    public void printCurrentCommands() {
        RobotLog.d("===== Current Commands =====");
        for (Map.Entry<Subsystem, Command> entry : activeSubsystemCommands.entrySet()) {
            RobotLog.d("Subsystem: " + entry.getKey().getClass().getSimpleName() +
                    ", Command: " + entry.getValue().getClass().getSimpleName());
        }
        RobotLog.d("============================");
    }

    private void cancel(Command command) {
        Subsystem requiredSubsystem = command.getRequiredSubsystem();
        if (requiredSubsystem != null) {
            activeSubsystemCommands.remove(requiredSubsystem);
        }

        command.end();
        scheduledCommands.remove(command);
        RobotLog.d("Command Cancelled: " + command.getClass().getSimpleName());
    }



    public boolean isCommandScheduled(Command command) {
        Subsystem subsystem = command.getRequiredSubsystem();
        return subsystem != null && activeSubsystemCommands.get(subsystem) == command;
    }


    public void removeDuplicateCommands() {
        List<Command> uniqueCommands = new ArrayList<>();

        for (Command command : new ArrayList<>(scheduledCommands)) {
            String name = command.getClass().getSimpleName();
            scheduledCommands.removeIf(c -> c.getClass().getSimpleName().equalsIgnoreCase(name));
            uniqueCommands.add(command);
        }

        scheduledCommands.addAll(uniqueCommands);
    }
}