
package org.firstinspires.ftc.teamcode.Commands.Pathing;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.JSONLoader;
import org.firstinspires.ftc.teamcode.Tools.Vector;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class FollowBrainStemPath implements Command {

    private final String pathName;
    private final Drive drive;

    private JSONArray waypoints;
    private int currentWaypoint;
    private boolean finished = false;

    private static final double INCH_TO_METER = 0.0254;

    public FollowBrainStemPath(String pathName, Drive drive) {
        this.pathName = pathName;
        this.drive = drive;
    }

    private double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    private double wrapAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private Pose2D getWaypointPose(int index) throws JSONException {
        JSONObject waypoint = waypoints.getJSONObject(index);

        double x = waypoint.getDouble("x") * INCH_TO_METER;
        double y = waypoint.getDouble("y") * INCH_TO_METER;
        double heading = Math.toRadians(
                waypoint.getDouble("rotation")
        );

        return new Pose2D(
                DistanceUnit.METER,
                x,
                y,
                AngleUnit.RADIANS,
                heading
        );
    }

    @Override
    public void start() {
        finished = false;
        currentWaypoint = 1;
        waypoints = null;

        drive.stopMotors();

        try {
            JSONLoader loader = new JSONLoader(AppUtil.getDefContext(), pathName);
            JSONObject path = loader.getJSON();

            waypoints = path.getJSONArray("waypoints");

            if (waypoints.length() < 2) {
                System.out.println("PATH ERROR: Not enough waypoints");
                finished = true;
                return;
            }

            Pose2D startingPose = getWaypointPose(0);
            drive.resetOdometry(startingPose);

            System.out.println("PATH STARTED: " + pathName);
            System.out.println("WAYPOINTS: " + waypoints.length());

        } catch (Exception e) {
            System.out.println("PATH LOAD ERROR: " + e);
            finished = true;
            drive.stopMotors();
        }
    }


    @Override
    public void execute() {
        if (finished || waypoints == null) {
            return;
        }

        try {
            Pose2D target = getWaypointPose(currentWaypoint);
            Pose2D current = drive.getPose();

            double targetX = target.getX(DistanceUnit.METER);
            double targetY = target.getY(DistanceUnit.METER);
            double targetHeading = target.getHeading(AngleUnit.RADIANS);

            double currentX = -current.getX(DistanceUnit.METER);
            double currentY = -current.getY(DistanceUnit.METER);
            double currentHeading = current.getHeading(AngleUnit.RADIANS);

            double errorX = targetX - currentX;
            double errorY = targetY - currentY;

            double headingError = wrapAngle(
                    targetHeading - currentHeading
            );

            double distance = Math.hypot(errorX, errorY);

            if (distance <= Constants.Drive.DISTANCE_TOLERANCE &&
                    Math.abs(headingError) <= Constants.Drive.ANGLE_TOLERANCE) {

                currentWaypoint++;

                if (currentWaypoint >= waypoints.length()) {
                    finished = true;
                    drive.stopMotors();
                    System.out.println("PATH FINISHED");
                }

                return;
            }

            // Field-relative PID outputs
            double xPower =
                    Constants.Drive.DrivePID.xController.calculate(
                            currentX, targetX
                    );

            double yPower =
                    Constants.Drive.DrivePID.yController.calculate(
                            currentY, targetY
                    );

            // Heading PID
            double rotationPower =
                    Constants.Drive.DrivePID.headingController.calculate(
                            -headingError, 0
                    );

            rotationPower = -Math.max(
                    -1,
                    Math.min(1, rotationPower)
            );


            double cos = Math.cos(currentHeading);
            double sin = Math.sin(currentHeading);

            double robotX = xPower * cos - yPower * sin;
            double robotY = xPower * sin + yPower * cos;

            robotX = clamp(robotX);
            robotY = clamp(robotY);

            drive.setMotors(
                    new Vector(-robotY, -robotX),
                    rotationPower
            );

            System.out.println(
                    "PATH | WP: " + currentWaypoint +
                            " | X: " + currentX +
                            " | Y: " + currentY +
                            " | Heading: " + currentHeading +
                            " | TargetX: " + targetX +
                            " | TargetY: " + targetY +
                            " | TargetHeading: " + targetHeading +
                            " | ErrorX: " + errorX +
                            " | ErrorY: " + errorY +
                            " | HeadingError: " + headingError +
                            " | RobotX: " + robotX +
                            " | RobotY: " + robotY +
                            " | Turn: " + rotationPower
            );

        } catch (Exception e) {
            System.out.println("PATH EXECUTION ERROR: " + e);
            e.printStackTrace();

            finished = true;
            drive.stopMotors();
        }
    }


    @Override
    public void end() {
        drive.stopMotors();
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public Subsystem getRequiredSubsystem() {
        return drive;
    }
}
