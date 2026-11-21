package org.firstinspires.ftc.teamcode.Commands.Pathing;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.JSONLoader;
import org.firstinspires.ftc.teamcode.Tools.Vector;
import org.json.JSONArray;
import org.json.JSONObject;

public class Follower implements Command {
    public static double KP_X = 2.0;
    public static double KP_Y = 2.0;
    public static double KD_X = 0.12;
    public static double KD_Y = 0.12;
    public static double KV_X = 0.50;
    public static double KV_Y = 0.50;
    public static double KP_HEADING = 2.0;
    public static double KD_HEADING = 0.12;
    public static double KV_HEADING = 0.12;

    public static double KS_TRANSLATION = 0.0;
    public static double KS_HEADING = 0.0;

    public static double PLAYBACK_RATE = 0.70;
    public static double MAX_TRANSLATION_POWER = 0.65;
    public static double MAX_TURN_POWER = 0.40;
    public static double MAX_WHEEL_POWER = 0.70;

    public static double SLOW_DISTANCE_M = 0.05;
    public static double PAUSE_DISTANCE_M = 0.18;
    public static double SLOW_HEADING_RAD = Math.toRadians(10);
    public static double PAUSE_HEADING_RAD = Math.toRadians(35);

    public static double POSITION_TOLERANCE_M = 0.015;
    public static double HEADING_TOLERANCE_RAD = Math.toRadians(2);
    public static double STOP_SPEED_MPS = 0.06;
    public static double STOP_OMEGA_RADPS = 0.15;
    public static double SETTLE_SECONDS = 0.25;
    public static double VELOCITY_FILTER_SECONDS = 0.06;
    public static double TIMEOUT_MULTIPLIER = 3.0;
    public static double EXTRA_TIMEOUT_SECONDS = 4.0;

    private static final double POSE_X_SIGN = -1.0;
    private static final double POSE_Y_SIGN = -1.0;
    private static final double HEADING_SIGN = 1.0;

    private final String pathName;
    private final Drive drive;
    private Sample[] samples;
    private int segment;
    private long startNanos;
    private long previousNanos;
    private double pathTime;
    private double timeoutSeconds;
    private double settleStarted = -1;
    private boolean havePreviousPose;
    private double previousX, previousY, previousHeading;
    private double velocityX, velocityY, angularVelocity;
    private boolean finished;
    private boolean successful;
    private String status = "Not started";

    private double currentX, currentY, currentHeading;
    private double targetX, targetY, targetHeading;
    private double positionError, headingError;
    private double progressRate;
    private double lastRobotX, lastRobotY, lastTurn;

    private static final class Sample {
        double time, x, y, heading, vx, vy, omega;
    }

    public Follower(String pathName, Drive drive) {
        this.pathName = pathName;
        this.drive = drive;
    }

    public boolean prepare() {
        try {
            String fileName = pathName;
            if (!fileName.endsWith(".polarpath") &&
                    !fileName.endsWith(".polarpaths") &&
                    !fileName.endsWith(".json")) {
                fileName += ".polarpath";
            }
            JSONObject json = new JSONLoader(AppUtil.getDefContext(), fileName).getJSON();
            JSONArray points = json.getJSONArray("sampled_points");
            if (points.length() < 2) {
                throw new IllegalArgumentException("Need at least two sampled_points");
            }
            Sample[] loaded = new Sample[points.length()];
            double firstTime = number(points.getJSONObject(0), "time");
            for (int i = 0; i < loaded.length; i++) {
                JSONObject point = points.getJSONObject(i);
                Sample s = new Sample();
                s.time = number(point, "time") - firstTime;
                if (!finite(s.time)) {
                    throw new IllegalArgumentException("Invalid relative sample time: " + i);
                }
                s.x = number(point, "x");
                s.y = number(point, "y");
                s.heading = wrap(number(point, "angle"));
                s.vx = number(point, "x_velocity");
                s.vy = number(point, "y_velocity");
                s.omega = number(point, "angular_velocity");
                if (i > 0 && s.time <= loaded[i - 1].time) {
                    throw new IllegalArgumentException("Sample times must strictly increase: " + i);
                }
                loaded[i] = s;
            }
            samples = loaded;
            finished = false;
            successful = false;
            status = "Ready: " + fileName;
            return true;
        } catch (Exception e) {
            samples = null;
            fail("Load error: " + e.getMessage());
            return false;
        }
    }

    @Override
    public void start() {
        drive.stopMotors();
        finished = false;
        successful = false;
        if (samples == null && !prepare()) return;
        try {
            if (!(PLAYBACK_RATE > 0 && PLAYBACK_RATE <= 1)) {
                throw new IllegalArgumentException("PLAYBACK_RATE must be > 0 and <= 1");
            }
            Sample first = samples[0];
            drive.resetOdometry(new Pose2D(
                    DistanceUnit.METER,
                    first.x / POSE_X_SIGN, first.y / POSE_Y_SIGN,
                    AngleUnit.RADIANS, first.heading / HEADING_SIGN));
            drive.periodic();

            segment = 0;
            pathTime = 0;
            settleStarted = -1;
            havePreviousPose = false;
            velocityX = velocityY = angularVelocity = 0;
            progressRate = 0;
            lastRobotX = lastRobotY = lastTurn = 0;
            timeoutSeconds = getDuration() / PLAYBACK_RATE * Math.max(1, TIMEOUT_MULTIPLIER)
                    + Math.max(0, EXTRA_TIMEOUT_SECONDS);
            previousNanos = startNanos = System.nanoTime();
            status = "Running";
        } catch (Exception e) {
            fail("Start error: " + e.getMessage());
        }
    }

    @Override
    public void execute() {
        if (finished || samples == null) return;
        try {
            long now = System.nanoTime();
            double dt = (now - previousNanos) / 1e9;
            if (dt < 0.005) return;
            previousNanos = now;
            double wallTime = (now - startNanos) / 1e9;

            Pose2D pose = drive.getPose();
            currentX = POSE_X_SIGN * pose.getX(DistanceUnit.METER);
            currentY = POSE_Y_SIGN * pose.getY(DistanceUnit.METER);
            currentHeading = wrap(HEADING_SIGN * pose.getHeading(AngleUnit.RADIANS));
            if (!finite(currentX) || !finite(currentY) || !finite(currentHeading)) {
                fail("Invalid odometry reading");
                return;
            }
            updateVelocity(dt);

            Sample before = sampleAt(pathTime);
            double distance = Math.hypot(before.x - currentX, before.y - currentY);
            double angle = Math.abs(wrap(before.heading - currentHeading));
            progressRate = clip(PLAYBACK_RATE, 0, 1) * Math.min(
                    gate(distance, SLOW_DISTANCE_M, PAUSE_DISTANCE_M),
                    gate(angle, SLOW_HEADING_RAD, PAUSE_HEADING_RAD));
            pathTime = Math.min(getDuration(), pathTime + Math.min(dt, 0.05) * progressRate);
            Sample target = sampleAt(pathTime);
            targetX = target.x;
            targetY = target.y;
            targetHeading = target.heading;

            boolean atEnd = pathTime >= getDuration();
            if (atEnd) progressRate = 0;
            double desiredVX = atEnd ? 0 : target.vx * progressRate;
            double desiredVY = atEnd ? 0 : target.vy * progressRate;
            double desiredOmega = atEnd ? 0 : target.omega * progressRate;
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            positionError = Math.hypot(errorX, errorY);
            headingError = wrap(targetHeading - currentHeading);

            double powerX = KP_X * errorX + KV_X * desiredVX
                    + KD_X * (desiredVX - velocityX);
            double powerY = KP_Y * errorY + KV_Y * desiredVY
                    + KD_Y * (desiredVY - velocityY);
            double turn = KP_HEADING * headingError + KV_HEADING * desiredOmega
                    + KD_HEADING * (desiredOmega - angularVelocity);

            double powerLength = Math.hypot(powerX, powerY);
            if (powerLength > 1e-9 && KS_TRANSLATION > 0) {
                double friction = KS_TRANSLATION * clip(positionError / 0.05
                        + Math.hypot(desiredVX, desiredVY) / 0.10, 0, 1);
                powerX += friction * powerX / powerLength;
                powerY += friction * powerY / powerLength;
            }
            if (Math.abs(turn) > 1e-9 && KS_HEADING > 0) {
                double friction = KS_HEADING * clip(Math.abs(headingError) / Math.toRadians(8)
                        + Math.abs(desiredOmega) / 0.25, 0, 1);
                turn += Math.copySign(friction, turn);
            }

            powerLength = Math.hypot(powerX, powerY);
            double translationLimit = clip(MAX_TRANSLATION_POWER, 0, 1);
            if (powerLength > translationLimit && powerLength > 0) {
                powerX *= translationLimit / powerLength;
                powerY *= translationLimit / powerLength;
            }
            turn = clip(turn, -clip(MAX_TURN_POWER, 0, 1), clip(MAX_TURN_POWER, 0, 1));

            double cos = Math.cos(currentHeading);
            double sin = Math.sin(currentHeading);
            double robotX = powerX * cos + powerY * sin;
            double robotY = -powerX * sin + powerY * cos;

            double demand = Math.abs(robotX) + Math.abs(robotY) + Math.abs(turn);
            double wheelLimit = clip(MAX_WHEEL_POWER, 0, 1);
            if (demand > wheelLimit && demand > 0) {
                double scale = wheelLimit / demand;
                robotX *= scale;
                robotY *= scale;
                turn *= scale;
            }
            if (!finite(robotX) || !finite(robotY) || !finite(turn)) {
                fail("Invalid controller output; check Dashboard settings");
                return;
            }

            boolean settled = atEnd && positionError <= POSITION_TOLERANCE_M
                    && Math.abs(headingError) <= HEADING_TOLERANCE_RAD
                    && Math.hypot(velocityX, velocityY) <= STOP_SPEED_MPS
                    && Math.abs(angularVelocity) <= STOP_OMEGA_RADPS;
            if (settled) {
                if (settleStarted < 0) settleStarted = wallTime;
                if (wallTime - settleStarted >= SETTLE_SECONDS) {
                    successful = true;
                    finished = true;
                    status = "Finished";
                    stopOutput();
                    return;
                }
            } else {
                settleStarted = -1;
            }
            if (wallTime >= timeoutSeconds) {
                fail("Timed out: check odometry, motor mapping, path speed and gains");
                return;
            }
            lastRobotX = robotX;
            lastRobotY = robotY;
            lastTurn = turn;
            sendRobotPower(robotX, robotY, turn);
        } catch (Exception e) {
            fail("Execution error: " + e.getMessage());
        }
    }

    private void sendRobotPower(double robotX, double robotY, double ccwPower) {
        drive.setMotors(new Vector(-robotY, -robotX), -ccwPower);
    }

    private void updateVelocity(double dt) {
        if (havePreviousPose) {
            double a = dt / (Math.max(0, VELOCITY_FILTER_SECONDS) + dt);
            velocityX += a * ((currentX - previousX) / dt - velocityX);
            velocityY += a * ((currentY - previousY) / dt - velocityY);
            angularVelocity += a * (wrap(currentHeading - previousHeading) / dt - angularVelocity);
        }
        previousX = currentX;
        previousY = currentY;
        previousHeading = currentHeading;
        havePreviousPose = true;
    }

    private Sample sampleAt(double time) {
        while (segment < samples.length - 2 && samples[segment + 1].time <= time) segment++;
        Sample a = samples[segment];
        Sample b = samples[segment + 1];
        double t = clip((time - a.time) / (b.time - a.time), 0, 1);
        Sample s = new Sample();
        s.x = lerp(a.x, b.x, t);
        s.y = lerp(a.y, b.y, t);
        s.heading = wrap(a.heading + wrap(b.heading - a.heading) * t);
        s.vx = lerp(a.vx, b.vx, t);
        s.vy = lerp(a.vy, b.vy, t);
        s.omega = lerp(a.omega, b.omega, t);
        return s;
    }

    private static double number(JSONObject json, String key) throws Exception {
        double value = json.getDouble(key);
        if (!finite(value)) throw new IllegalArgumentException("Non-finite " + key);
        return value;
    }

    private static double gate(double error, double slow, double pause) {
        return clip((pause - error) / Math.max(1e-6, pause - slow), 0, 1);
    }

    private static boolean finite(double v) { return !Double.isNaN(v) && !Double.isInfinite(v); }
    private static double clip(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double wrap(double a) { return Math.atan2(Math.sin(a), Math.cos(a)); }
    private static double lerp(double a, double b, double t) { return a + (b - a) * t; }


    private void stopOutput() {
        lastRobotX = lastRobotY = lastTurn = 0;
        drive.stopMotors();
    }

    private void fail(String message) {
        status = message;
        successful = false;
        finished = true;
        stopOutput();
        System.out.println("FOLLOWER: " + message);
    }

    @Override
    public void end() {
        stopOutput();
        if (!finished) status = "Stopped";
        finished = true;
    }

    @Override public boolean isFinished() { return finished; }
    @Override public Subsystem getRequiredSubsystem() { return drive; }

    public boolean isSuccessful() { return successful; }
    public String getStatus() { return status; }
    public double getPathTime() { return pathTime; }
    public double getDuration() { return samples == null ? 0 : samples[samples.length - 1].time; }
    public double getCurrentX() { return currentX; }
    public double getCurrentY() { return currentY; }
    public double getCurrentHeading() { return currentHeading; }
    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
    public double getTargetHeading() { return targetHeading; }
    public double getPositionError() { return positionError; }
    public double getHeadingError() { return headingError; }
    public double getProgressRate() { return progressRate; }
    public double getRobotXPower() { return lastRobotX; }
    public double getRobotYPower() { return lastRobotY; }
    public double getTurnPower() { return lastTurn; }
}
