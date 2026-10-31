package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.PIDF;

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
            public static final String SHOOTER_NAME = "ShooterMotor";
            public static final String HOOD_NAME = "HoodServo";
            public static final String LIMELIGHT_NAME_TAG_DETECTION = "limelight";
        }
        public static class Intake {
            public static final String INTAKE_NAME = "intake_motor";
        }

        public static class Indexer {
            public static final String INDEXER_NAME = "IndexerMotor";
        }

    }

    public static class Physical {
        public static final double TOP_SPEED = 1;
        public static final double MAX_ANGULAR_SPEED = -1;
       public static final SparkFunOTOS.Pose2D MOUSE_SENSOR_MOUNTED_POSE = new SparkFunOTOS.Pose2D(0.1524, -0.1016, 0);
       public static final double MOUSE_SENSOR_LINEAR_SCALAR = 1.525985719;
       public static final double MOUSE_SENSOR_ANGULAR_SCALAR = 1.01530630663;

       public static class Ratios {
           public static final double SHOOTER_TICKS_PER_REV = 28;
           public static final double SHOOTER_GEAR_RATIO = 1/1;
       }
       public static class Shooter {
           private static final double SHOOTER_SPEED = 1;
           public static final double SHOOTER_RPM = 2500;
           private static final double FEED_FORWARD = SHOOTER_SPEED / SHOOTER_RPM ;
           public static final PIDF VELOCITY_PID = new PIDF(1, 0, 0, FEED_FORWARD);
       }
    }

    public static class Drive {
        public static final double DISTANCE_TOLERANCE = 0.01;
        public static final double ANGLE_TOLERANCE = 0.1;
        public static final int LIMELIGHT_OBJECT_DETECTION_PIPELINE = 1;

        public static class DrivePID {
            public static final PID xController = new PID(1.5, 0, 0); // Tune
            public static final PID yController = new PID(1.5, 0, 0); // Tune
            public static final PID headingController = new PID(1, 0, 0); // Tune
        }

        public static class PurePursuit {
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
            public static boolean RESET_ODOMETRY_ON_START = true;

        }
    }


    private static double inchesToMeters(double inches) {
        return inches / 39.37;
    }

}
