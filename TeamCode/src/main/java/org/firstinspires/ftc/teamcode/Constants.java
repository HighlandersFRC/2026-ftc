package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class Constants {

    public static class InitInfo {
        public static class Drive {
            public static final String LIMELIGHT_NAME_OBJECT_DETECTION = "limelight";
            public static final String MOUSE_NAME = "mouse";
            public static final String FRONT_RIGHT_NAME = "FrontRightMotor";
            public static final String FRONT_LEFT_NAME = "FrontLeftMotor";
            public static final String BACK_RIGHT_NAME = "BackRightMotor";
            public static final String BACK_LEFT_NAME = "BackLeftMotor";
        }

        public static class Shooter {
            public static final String SHOOTER_NAME = "ShooterMotor";
            public static final String HOOD_NAME = "HoodServo";
            public static final String LIMELIGHT_NAME_TAG_DETECTION = "limelight";
        }
        public static class Intake {
            public static final String INTAKE_NAME = "IntakeMotor";
        }

        public static class Indexer {
            public static final String INDEXER_NAME = "IndexerMotor";
        }

    }

    public static class Physical {
        public static final double TOP_SPEED = 0;
        public static final double MAX_ANGULAR_SPEED = 0;
       public static final SparkFunOTOS.Pose2D MOUSE_SENSOR_MOUNTED_POSE = new SparkFunOTOS.Pose2D(0, 0, 0);
       public static final double MOUSE_SENSOR_LINEAR_SCALAR = 0.0;
       public static final double MOUSE_SENSOR_ANGULAR_SCALAR = 0.0;
    }

    public static class Drive {
        public static final double DISTANCE_TOLERANCE = 0.1;
        public static final double ANGLE_TOLERANCE = 0.1;
        public static final int LIMELIGHT_OBJECT_DETECTION_PIPELINE = 1;
    }


}
