package org.firstinspires.ftc.teamcode.Commands.Pathing.PurePursuit;


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

import static org.firstinspires.ftc.teamcode.Commands.Pathing.Follower.KD_X;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.CURVE_SLOWDOWN_M;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.HEADING_PAUSE_RAD;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.HEADING_SLOW_RAD;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.HEADING_TOLERANCE_RAD;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.KD_HEADING;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.KD_Y;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.KP_HEADING;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.KP_X;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.KP_Y;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.LOOKAHEAD_M;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.MAX_TRANSLATION_POWER;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.MAX_TURN_POWER;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.MAX_WHEEL_POWER;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.POSITION_TOLERANCE_M;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.PROJECTION_WINDOW_M;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.RESET_ODOMETRY_ON_START;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.SETTLE_SECONDS;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.STOP_OMEGA_RADPS;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.STOP_SPEED_MPS;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.TIMEOUT_SECONDS;
import static org.firstinspires.ftc.teamcode.Constants.Drive.PurePursuit.VELOCITY_FILTER_SECONDS;

import java.util.ArrayList;

public class PurepursuitFollower implements Command {

    private static final double POSE_X_SIGN = -1.0;
    private static final double POSE_Y_SIGN = -1.0;
    private static final double HEADING_SIGN = 1.0;
    private static final double EPS = 1e-9;
    private static final double SAME_POSITION_M = 1e-6;
    
    private final String pathName;
    private final Drive drive;
    private PathPoint[] points;
    private Phase[] phases;
    private int phaseIndex;
    private double progress;
    private long startNanos;
    private long previousNanos;
    private double settleStarted = -1;
    private boolean havePreviousPose;
    private boolean finished = true;
    private boolean successful;
    private String status = "Not started";

    private double currentX, currentY, currentHeading;
    private double previousX, previousY, previousWrappedHeading;
    private double velocityX, velocityY, angularVelocity;
    private double targetX, targetY, targetHeading;
    private double crossTrackError, positionError, headingError;
    private double lastRobotX, lastRobotY, lastTurn;

    private static final class Phase {
        final int start, end;
        final boolean rotationOnly;

        Phase(int start, int end, boolean rotationOnly) {
            this.start = start;
            this.end = end;
            this.rotationOnly = rotationOnly;
        }
    }

    public PurepursuitFollower(String pathName, Drive drive) {
        if (drive == null) throw new IllegalArgumentException("drive is null");
        this.pathName = pathName;
        this.drive = drive;
    }

    public JSONObject loadPath(String name) {
        if (name == null || name.trim().isEmpty()) {
            throw new IllegalArgumentException("Path name is empty");
        }
        name = name.trim();
        if (!name.endsWith(".polarpath") && !name.endsWith(".polarpaths")
                && !name.endsWith(".json")) {
            name += ".polarpath";
        }
        return new JSONLoader(AppUtil.getDefContext(), name).getJSON();
    }

    public boolean prepare() {
        stopOutput();
        finished = true;
        successful = false;
        try {
            JSONArray input = loadPath(pathName).getJSONArray("sampled_points");
            if (input.length() == 0) {
                throw new IllegalArgumentException("sampled_points is empty");
            }
            ArrayList<PathPoint> loaded = new ArrayList<>();
            double distance = 0;
            for (int i = 0; i < input.length(); i++) {
                JSONObject row = input.getJSONObject(i);
                double x = number(row, "x");
                double y = number(row, "y");
                double heading = wrap(number(row, "angle"));
                if (!loaded.isEmpty()) {
                    PathPoint previous = loaded.get(loaded.size() - 1);
                    heading = previous.getW() + wrap(heading - previous.getW());
                    double length = Math.hypot(x - previous.getX(), y - previous.getY());
                    if (!finite(length)) throw new IllegalArgumentException("Path is too large");
                    if (length <= SAME_POSITION_M) {
                        x = previous.getX();
                        y = previous.getY();
                        length = 0;
                        if (Math.abs(heading - previous.getW()) < EPS) continue;
                    }
                    distance += length;
                    if (!finite(distance)) throw new IllegalArgumentException("Path is too large");
                }
                loaded.add(new PathPoint(x, y, heading, 0, distance));
            }
            points = loaded.toArray(new PathPoint[0]);
            calculateCurvatures();
            buildPhases();
            finished = true;
            successful = false;
            status = "Ready: " + points.length + " points, " + phases.length + " phases";
            return true;
        } catch (Exception e) {
            points = null;
            phases = null;
            fail("Load error: " + e.getMessage());
            return false;
        }
    }

    private void calculateCurvatures() {
        for (int i = 1; i < points.length - 1; i++) {
            PathPoint a = points[i - 1], b = points[i], c = points[i + 1];
            double ab = Math.hypot(b.getX() - a.getX(), b.getY() - a.getY());
            double bc = Math.hypot(c.getX() - b.getX(), c.getY() - b.getY());
            double ac = Math.hypot(c.getX() - a.getX(), c.getY() - a.getY());
            double cross = (b.getX() - a.getX()) * (c.getY() - a.getY())
                    - (b.getY() - a.getY()) * (c.getX() - a.getX());
            if (ab > SAME_POSITION_M && bc > SAME_POSITION_M && ac > SAME_POSITION_M) {
                b.setNewCurvature(2 * Math.abs(cross) / (ab * bc * ac));
            }
        }
    }

    private void buildPhases() {
        ArrayList<Phase> result = new ArrayList<>();
        int start = 0;
        while (start < points.length - 1) {
            boolean rotation = points[start + 1].getDistance() == points[start].getDistance();
            int end = start + 1;
            while (end < points.length - 1) {
                boolean nextRotation = points[end + 1].getDistance() == points[end].getDistance();
                if (nextRotation != rotation) break;
                if (rotation && (points[end].getW() - points[end - 1].getW())
                        * (points[end + 1].getW() - points[end].getW()) < 0) break;
                if (!rotation && isCusp(end)) break;
                end++;
            }
            result.add(new Phase(start, end, rotation));
            start = end;
        }
        if (result.isEmpty()) result.add(new Phase(0, 0, true));
        phases = result.toArray(new Phase[0]);
    }

    private boolean isCusp(int i) {
        PathPoint a = points[i - 1], b = points[i], c = points[i + 1];
        double ax = b.getX() - a.getX(), ay = b.getY() - a.getY();
        double bx = c.getX() - b.getX(), by = c.getY() - b.getY();
        return ax * bx + ay * by < -0.95 * Math.hypot(ax, ay) * Math.hypot(bx, by);
    }

    @Override
    public void start() {
        stopOutput();
        if (points == null && !prepare()) return;
        finished = false;
        successful = false;
        try {
            validateSettings();
            PathPoint first = points[0];
            if (RESET_ODOMETRY_ON_START) {
                drive.resetOdometry(new Pose2D(DistanceUnit.METER,
                        first.getX() / POSE_X_SIGN, first.getY() / POSE_Y_SIGN,
                        AngleUnit.RADIANS, wrap(first.getW()) / HEADING_SIGN));
            }
            drive.periodic();
            phaseIndex = 0;
            progress = 0;
            settleStarted = -1;
            havePreviousPose = false;
            velocityX = velocityY = angularVelocity = 0;
            currentHeading = first.getW();
            readPose(0);
            targetX = first.getX();
            targetY = first.getY();
            targetHeading = first.getW();
            previousNanos = startNanos = System.nanoTime();
            status = "Running";
        } catch (Exception e) {
            fail("Start error: " + e.getMessage());
        }
    }

    @Override
    public void execute() {
        if (finished || points == null) return;
        try {
            long now = System.nanoTime();
            double dt = (now - previousNanos) / 1e9;
            if (dt < 0.005) return;
            previousNanos = now;
            double elapsed = (now - startNanos) / 1e9;
            validateSettings();
            if (elapsed >= TIMEOUT_SECONDS) {
                fail("Timed out; check pose, motor mapping, path units and gains");
                return;
            }
            readPose(dt);
            Phase phase = phases[phaseIndex];
            PathPoint end = points[phase.end];
            positionError = Math.hypot(end.getX() - currentX, end.getY() - currentY);

            PathPoint target;
            double remaining;
            boolean approachingEnd;
            if (phase.rotationOnly) {
                progress = end.getDistance();
                remaining = 0;
                target = end;
                targetHeading = end.getW();
                crossTrackError = positionError;
                approachingEnd = true;
            } else {
                updateProgress(phase);
                remaining = Math.max(0, end.getDistance() - progress);
                target = findLookahead(phase);
                approachingEnd = remaining <= LOOKAHEAD_M && positionError <= LOOKAHEAD_M;
                targetHeading = approachingEnd ? end.getW() : pointAt(progress, phase).getW();
            }
            targetX = target.getX();
            targetY = target.getY();
            headingError = targetHeading - currentHeading;

            double powerX = KP_X * (targetX - currentX) - KD_X * velocityX;
            double powerY = KP_Y * (targetY - currentY) - KD_Y * velocityY;
            double turn = KP_HEADING * headingError - KD_HEADING * angularVelocity;

            double translationLimit = MAX_TRANSLATION_POWER;
            if (!phase.rotationOnly && !approachingEnd) {
                double curvature = Math.max(target.getCurvature(),
                        pointAt(progress, phase).getCurvature());
                translationLimit /= 1 + CURVE_SLOWDOWN_M * curvature;
                translationLimit *= clip((HEADING_PAUSE_RAD - Math.abs(headingError))
                        / (HEADING_PAUSE_RAD - HEADING_SLOW_RAD), 0, 1);
            }
            double length = Math.hypot(powerX, powerY);
            if (length > translationLimit && length > EPS) {
                powerX *= translationLimit / length;
                powerY *= translationLimit / length;
            }
            turn = clip(turn, -MAX_TURN_POWER, MAX_TURN_POWER);

            double cos = Math.cos(currentHeading), sin = Math.sin(currentHeading);
            double robotX = powerX * cos + powerY * sin;
            double robotY = -powerX * sin + powerY * cos;
            double demand = Math.abs(robotX) + Math.abs(robotY) + Math.abs(turn);
            if (demand > MAX_WHEEL_POWER) {
                double scale = MAX_WHEEL_POWER / demand;
                robotX *= scale;
                robotY *= scale;
                turn *= scale;
            }
            if (!finite(robotX) || !finite(robotY) || !finite(turn)) {
                throw new IllegalArgumentException("Non-finite controller output");
            }

            boolean settled = remaining <= POSITION_TOLERANCE_M
                    && positionError <= POSITION_TOLERANCE_M
                    && Math.abs(end.getW() - currentHeading) <= HEADING_TOLERANCE_RAD
                    && Math.hypot(velocityX, velocityY) <= STOP_SPEED_MPS
                    && Math.abs(angularVelocity) <= STOP_OMEGA_RADPS;
            if (settled) {
                if (settleStarted < 0) settleStarted = elapsed;
                if (elapsed - settleStarted >= SETTLE_SECONDS) {
                    if (phaseIndex == phases.length - 1) {
                        successful = true;
                        finished = true;
                        status = "Finished";
                    } else {
                        phaseIndex++;
                        progress = points[phases[phaseIndex].start].getDistance();
                        settleStarted = -1;
                    }
                    stopOutput();
                    return;
                }
            } else {
                settleStarted = -1;
            }
            status = (phase.rotationOnly ? "Turning" : "Following")
                    + " (phase " + (phaseIndex + 1) + "/" + phases.length + ")";
            lastRobotX = robotX;
            lastRobotY = robotY;
            lastTurn = turn;
            sendRobotPower(robotX, robotY, turn);
        } catch (Exception e) {
            fail("Execution error: " + e.getMessage());
        }
    }

    private void updateProgress(Phase phase) {
        double limit = Math.min(points[phase.end].getDistance(),
                progress + PROJECTION_WINDOW_M);
        double bestDistanceSquared = Double.POSITIVE_INFINITY;
        double bestProgress = progress;
        for (int i = segmentAt(progress, phase); i < phase.end; i++) {
            PathPoint a = points[i], b = points[i + 1];
            if (a.getDistance() > limit) break;
            double segmentLength = b.getDistance() - a.getDistance();
            double dx = b.getX() - a.getX(), dy = b.getY() - a.getY();
            double lo = clip((progress - a.getDistance()) / segmentLength, 0, 1);
            double hi = clip((limit - a.getDistance()) / segmentLength, 0, 1);
            double t = clip(((currentX - a.getX()) * dx + (currentY - a.getY()) * dy)
                    / (dx * dx + dy * dy), lo, hi);
            double ex = a.getX() + dx * t - currentX;
            double ey = a.getY() + dy * t - currentY;
            double distanceSquared = ex * ex + ey * ey;
            if (distanceSquared < bestDistanceSquared - 1e-12) {
                bestDistanceSquared = distanceSquared;
                bestProgress = a.getDistance() + segmentLength * t;
            }
        }
        progress = Math.max(progress, bestProgress);
        crossTrackError = Math.sqrt(bestDistanceSquared);
    }

    private PathPoint findLookahead(Phase phase) {
        PathPoint closest = pointAt(progress, phase);
        if (Math.hypot(closest.getX() - currentX, closest.getY() - currentY) >= LOOKAHEAD_M) {
            return closest;
        }
        double limit = Math.min(points[phase.end].getDistance(), progress + 2 * LOOKAHEAD_M);
        for (int i = segmentAt(progress, phase); i < phase.end; i++) {
            PathPoint a = points[i], b = points[i + 1];
            if (a.getDistance() > limit) break;
            double dx = b.getX() - a.getX(), dy = b.getY() - a.getY();
            double fx = a.getX() - currentX, fy = a.getY() - currentY;
            double aa = dx * dx + dy * dy;
            double bb = 2 * (fx * dx + fy * dy);
            double cc = fx * fx + fy * fy - LOOKAHEAD_M * LOOKAHEAD_M;
            double discriminant = bb * bb - 4 * aa * cc;
            if (discriminant < 0) continue;
            double t = (-bb + Math.sqrt(Math.max(0, discriminant))) / (2 * aa);
            double s = a.getDistance() + t * (b.getDistance() - a.getDistance());
            if (t >= -EPS && t <= 1 + EPS && s > progress + EPS && s <= limit + EPS) {
                return interpolate(a, b, clip(t, 0, 1));
            }
        }
        double ahead = Math.min(points[phase.end].getDistance(), progress + LOOKAHEAD_M);
        PathPoint fallback = pointAt(ahead, phase);
        if (ahead > progress + POSITION_TOLERANCE_M
                && Math.hypot(fallback.getX() - currentX, fallback.getY() - currentY) < SAME_POSITION_M) {
            fallback = pointAt((progress + ahead) * 0.5, phase);
        }
        return fallback;
    }

    private int segmentAt(double s, Phase phase) {
        int lo = phase.start, hi = phase.end - 1;
        while (lo < hi) {
            int mid = (lo + hi) >>> 1;
            if (points[mid + 1].getDistance() <= s) lo = mid + 1;
            else hi = mid;
        }
        return lo;
    }

    private PathPoint pointAt(double s, Phase phase) {
        if (phase.start == phase.end) return points[phase.end];
        int i = segmentAt(s, phase);
        PathPoint a = points[i], b = points[i + 1];
        double t = clip((s - a.getDistance()) / (b.getDistance() - a.getDistance()), 0, 1);
        return interpolate(a, b, t);
    }

    private static PathPoint interpolate(PathPoint a, PathPoint b, double t) {
        return new PathPoint(lerp(a.getX(), b.getX(), t), lerp(a.getY(), b.getY(), t),
                lerp(a.getW(), b.getW(), t), lerp(a.getCurvature(), b.getCurvature(), t),
                lerp(a.getDistance(), b.getDistance(), t));
    }

    private void readPose(double dt) {
        Pose2D pose = drive.getPose();
        currentX = POSE_X_SIGN * pose.getX(DistanceUnit.METER);
        currentY = POSE_Y_SIGN * pose.getY(DistanceUnit.METER);
        double heading = wrap(HEADING_SIGN * pose.getHeading(AngleUnit.RADIANS));
        if (!finite(currentX) || !finite(currentY) || !finite(heading)) {
            throw new IllegalArgumentException("Invalid odometry reading");
        }
        if (havePreviousPose && dt > 0) {
            double deltaHeading = wrap(heading - previousWrappedHeading);
            currentHeading += deltaHeading;
            double alpha = dt / (VELOCITY_FILTER_SECONDS + dt);
            velocityX += alpha * ((currentX - previousX) / dt - velocityX);
            velocityY += alpha * ((currentY - previousY) / dt - velocityY);
            angularVelocity += alpha * (deltaHeading / dt - angularVelocity);
        } else {
            currentHeading += wrap(heading - currentHeading);
        }
        previousX = currentX;
        previousY = currentY;
        previousWrappedHeading = heading;
        havePreviousPose = true;
    }

    private void sendRobotPower(double robotX, double robotY, double ccwPower) {
        drive.setMotors(new Vector(-robotY, -robotX), -ccwPower);
    }

    private static void validateSettings() {
        double[] positive = {LOOKAHEAD_M, PROJECTION_WINDOW_M, KP_X, KP_Y, KP_HEADING,
                POSITION_TOLERANCE_M, HEADING_TOLERANCE_RAD, STOP_SPEED_MPS,
                STOP_OMEGA_RADPS, TIMEOUT_SECONDS};
        for (double value : positive) {
            if (!finite(value) || value <= 0) throw new IllegalArgumentException("Invalid positive Dashboard setting");
        }
        double[] nonnegative = {KD_X, KD_Y, KD_HEADING, CURVE_SLOWDOWN_M,
                SETTLE_SECONDS, VELOCITY_FILTER_SECONDS, HEADING_SLOW_RAD};
        for (double value : nonnegative) {
            if (!finite(value) || value < 0) throw new IllegalArgumentException("Invalid nonnegative Dashboard setting");
        }
        double[] powers = {MAX_TRANSLATION_POWER, MAX_TURN_POWER, MAX_WHEEL_POWER};
        for (double value : powers) {
            if (!finite(value) || value <= 0 || value > 1) {
                throw new IllegalArgumentException("Power limits must be > 0 and <= 1");
            }
        }
        if (!finite(HEADING_PAUSE_RAD) || HEADING_PAUSE_RAD <= HEADING_SLOW_RAD) {
            throw new IllegalArgumentException("HEADING_PAUSE_RAD must exceed HEADING_SLOW_RAD");
        }
    }

    private static double number(JSONObject json, String key) throws Exception {
        double value = json.getDouble(key);
        if (!finite(value)) throw new IllegalArgumentException("Non-finite " + key);
        return value;
    }

    private static boolean finite(double v) { return !Double.isNaN(v) && !Double.isInfinite(v); }
    private static double wrap(double a) { return Math.atan2(Math.sin(a), Math.cos(a)); }
    private static double clip(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double lerp(double a, double b, double t) { return a + (b - a) * t; }

    private void stopOutput() {
        lastRobotX = lastRobotY = lastTurn = 0;
        drive.stopMotors();
    }

    private void fail(String message) {
        successful = false;
        finished = true;
        status = message;
        stopOutput();
        System.out.println("PURE PURSUIT: " + message);
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
    public double getProgressMeters() { return progress; }
    public double getLengthMeters() { return points == null ? 0 : points[points.length - 1].getDistance(); }
    public double getCurrentX() { return currentX; }
    public double getCurrentY() { return currentY; }
    public double getCurrentHeading() { return currentHeading; }
    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
    public double getTargetHeading() { return targetHeading; }
    public double getCrossTrackError() { return crossTrackError; }
    public double getPositionError() { return positionError; }
    public double getHeadingError() { return headingError; }
    public double getRobotXPower() { return lastRobotX; }
    public double getRobotYPower() { return lastRobotY; }
    public double getTurnPower() { return lastTurn; }
}
