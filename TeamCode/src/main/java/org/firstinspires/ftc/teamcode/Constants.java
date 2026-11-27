package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Tools.PID;

public class Constants {

    public static class InitInfo {
        public static class Drive {
            public static final String LIMELIGHT_NAME_OBJECT_DETECTION = "limelight";
            public static final String MOUSE_NAME = "mouse";
            public static final String FRONT_RIGHT_NAME = "right_front";
            public static final String FRONT_LEFT_NAME = "left_front";
            public static final String BACK_RIGHT_NAME = "right_back";
            public static final String BACK_LEFT_NAME = "left_back";
        }

        public static class Shooter {
            public static final String SHOOTER_NAME = "shooter_motor";
            public static final String HOOD_NAME = "hood_servo";
            public static final String LIMELIGHT_NAME_TAG_DETECTION = "limelight";
        }
        public static class Intake {
            public static final String INTAKE_NAME = "intake_motor";
        }


    }

    public static class Field {

        public static boolean TARGET_TOP_HIVE(Pose2D currentPose) {
            double currentY = currentPose.getY(DistanceUnit.METER);

            return currentY >= 0;
        }

//                                                                                  TUNE                    TUNE
        public static final Pose2D BLUE_TOP_HIVE = new Pose2D(DistanceUnit.METER, inchesToMeters(12.5), inchesToMeters(55.0), AngleUnit.RADIANS, (3 * Math.PI) / 2);
        public static final Pose2D BLUE_BOTTOM_HIVE = new Pose2D(DistanceUnit.METER, BLUE_TOP_HIVE.getX(DistanceUnit.METER), -BLUE_TOP_HIVE.getY(DistanceUnit.METER), AngleUnit.RADIANS, BLUE_TOP_HIVE.getHeading(AngleUnit.RADIANS) + Math.PI);
        public static final Pose2D RED_TOP_HIVE = new Pose2D(DistanceUnit.METER, BLUE_TOP_HIVE.getX(DistanceUnit.METER) + inchesToMeters(25.0), BLUE_TOP_HIVE.getY(DistanceUnit.METER), AngleUnit.RADIANS, BLUE_TOP_HIVE.getHeading(AngleUnit.RADIANS));
        public static final Pose2D RED_BOTTOM_HIVE = new Pose2D(DistanceUnit.METER, RED_TOP_HIVE.getX(DistanceUnit.METER), -BLUE_TOP_HIVE.getY(DistanceUnit.METER), AngleUnit.RADIANS, BLUE_TOP_HIVE.getHeading(AngleUnit.RADIANS) + Math.PI);


    }


    public static class Physical {
       public static final double TOP_SPEED = 1.0;
       public static final double MAX_ANGULAR_SPEED = -1.0;

       public static final SparkFunOTOS.Pose2D MOUSE_SENSOR_MOUNTED_POSE = new SparkFunOTOS.Pose2D(-0.2159, 0.1016, -90);
       public static final double MOUSE_SENSOR_LINEAR_SCALAR = 1.0;
       public static final double MOUSE_SENSOR_ANGULAR_SCALAR = 1.0;
       public static final double GOBUILDA_MOTOR_TICKS_PER_REV = 28.0;
    }

    public static class Catapult {
        public static final double CATAPULT_MOTOR_ZERO = 0.0;
        public static final double CATAPULT_MOTOR_UP = 1.0; // Tune
        public static final double CATAPULT_TOLERANCE = 10.0; // Tune

        public static final PID CATAPULT_TARGET_PID = new PID(0.0, 0.0, 0.0); // Tune            
    }

    public static class Intake {
        public static final double INTAKE_SPEED = 100.0;
        public static final double OUTTAKE_SPEED = INTAKE_SPEED * -1.0; // This is for tuning just in case the outtake is diff than intake
        public static final double DEFAULT_SPEED = 0.0;
    }


    public static class Drive {
        public static final double DISTANCE_TOLERANCE = 0.01;
        public static final double ANGLE_TOLERANCE = 0.1;
        public static final int LIMELIGHT_OBJECT_DETECTION_PIPELINE = 1;

        public static final String[] OBJECTS = {
            "Red", // Red Nectar
            "Blue", // Blue Nectar
            "Yellow" // Pollen
        };

        public static class DrivePID {
            public static final PID xController = new PID(1.5, 0, 0); // Tune
            public static final PID yController = new PID(1.5, 0, 0); // Tune
            public static final PID headingController = new PID(1, 0, 0); // Tune
        }

        public static class PurePursuit {
            public static boolean RESET_ODOMETRY_ON_START = true; // Just to get the Mouse Sensor on the same page as the path

            public static double LOOKAHEAD_M = 0.20;
            public static double PROJECTION_WINDOW_M = 0.35;

            public static double KP_X = 2.0;
            public static double KP_Y = 2.0;
            public static double KD_X = 0.12;
            public static double KD_Y = 0.12;
            public static double KP_HEADING = 2.0;
            public static double KD_HEADING = 0.12;

            public static double MAX_TRANSLATION_POWER = 1.0;
            public static double MAX_TURN_POWER = 0.35;
            public static double MAX_WHEEL_POWER = 0.60;

            public static double CURVE_SLOWDOWN_M = 0.03;
            public static double HEADING_SLOW_RAD = Math.toRadians(25);
            public static double HEADING_PAUSE_RAD = Math.toRadians(75);

            public static double POSITION_TOLERANCE_M = 0.02;
            public static double HEADING_TOLERANCE_RAD = Math.toRadians(3);
            public static double STOP_SPEED_MPS = 0.06;
            public static double STOP_OMEGA_RADPS = 0.15;
            public static double SETTLE_SECONDS = 0.20;
            public static double VELOCITY_FILTER_SECONDS = 0.06;
            public static double TIMEOUT_SECONDS = 30.0;
        }
    }




    private static double inchesToMeters(double inches) {
        return inches / 39.37;
    }

    public static boolean withinTolerance(double targetPos, double currentPos, double tolerance) {
        return Math.abs(currentPos - targetPos) <= tolerance;
    }

    public static boolean validatePoseWithinTolerance(Pose2D targetPose, Pose2D currentPose, double translationTolerance, double angleTolerance) {
        boolean validX = withinTolerance(targetPose.getX(DistanceUnit.METER), currentPose.getX(DistanceUnit.METER), translationTolerance);
        boolean validY = withinTolerance(targetPose.getY(DistanceUnit.METER), currentPose.getY(DistanceUnit.METER), translationTolerance);
        boolean validHeading = withinTolerance(targetPose.getHeading(AngleUnit.RADIANS), currentPose.getHeading(AngleUnit.RADIANS), angleTolerance);

        return validX && validY && validHeading;
    }

}
