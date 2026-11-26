package org.firstinspires.ftc.teamcode.Commands.Pathing.PurePursuit;

public class Phase {
    final int start, end;
    final boolean rotationOnly;

    Phase(int start, int end, boolean rotationOnly) {
        this.start = start;
        this.end = end;
        this.rotationOnly = rotationOnly;
    }
}