package frc.robot.subsystems.swerve;

public class SwerveConstants {
        // swerve constants
        public final static double WIDTH = 0.545;
        public final static double LENGTH = 0.545;
        public final static double RADIUS = Math.sqrt(
                        Math.pow(WIDTH, 2) + Math.pow(LENGTH, 2)) / 2.0;

        // Modules constants
        public final static double TURNING_GEAR_RATIO = 150d / 7;
        private final static double DRIVE_GEAR_RATIO = 6.12;
        private final static double WHEEL_RADIUS = 0.05;

        public final static double VELOCITY_TIME_UNIT_IN_SECONDS = 1;

        public final static double DISTANCE_PER_PULSE = ((2 * WHEEL_RADIUS * Math.PI)
                        / DRIVE_GEAR_RATIO) * 0.97765363;
        public final static double ANGLE_PER_PULSE = 360d / TURNING_GEAR_RATIO;

        // front left module
        public final static double FRONT_LEFT_MODULE_OFFSET_ENCODER = 90;
        public final static boolean FRONT_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = true;
        public final static boolean FRONT_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean FRONT_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // front right module
        public final static double FRONT_RIGHT_MODULE_OFFSET_ENCODER = 335.4;
        public final static boolean FRONT_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = false;
        public final static boolean FRONT_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean FRONT_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // rear left module
        public final static double REAR_LEFT_MODULE_OFFSET_ENCODER = 102.21;
        public final static boolean REAR_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = true;
        public final static boolean REAR_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean REAR_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // rear right module
        public final static double REAR_RIGHT_MODULE_OFFSET_ENCODER = 101.77;
        public final static boolean REAR_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = false;
        public final static boolean REAR_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean REAR_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // Modules turning config
        // PID
        public final static double TURNING_PID_KP = 3.6;
        public final static double TURNING_PID_KI = 0;
        public final static double TURNING_PID_KD = 0;
        // Ramp
        public final static double OPEN_LOOP_RAMP = 0.25;
        public final static double CLOSED_LOOP_RAMP = 0;
        // Current Limit
        public final static int TURNING_CONTINUOUS_CURRENT_LIMIT = 25;
        public final static int TURNING_PEAK_CURRENT_LIMIT = 400;
        public final static double TURNING_PEAK_CURRENTDURATION = 0.1;
        public final static boolean turningEnableCurrentLimit = true;

        // Modules drive config
        // PID
        public final static double DRIVE_PID_KP = 3.1;
        public final static double DRIVE_PID_KI = 0;
        public final static double DRIVE_PID_KD = 0;
        public final static double DRIVE_KS = 0;
        public final static double DRIVE_KV = 0.18 * 12;
        // Current Limit
        public final static int DRIVE_CONTINUOS_CURRENT_LIMIT = 35;
        public final static int DRIVE_PEAK_CURRENT_LIMIT = 400;
        public final static double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public final static boolean DRIVE_ENBLE_CURRENT_LIMIT = true;

        // swerve physics
        public final static double MAX_VELOCITY = 5.3;
        public final static double MAX_ACCELERATION = 10.91;
        public final static double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / RADIUS; // radians

        // swerve controllers

        // swerve CONTROLLER
        public final static double KP_TRANSLATION = 9; // 3.3;
        public final static double KI_TRANSLATION = 0; //0.0009;

        // swerve theta PID_CONTROLLER radians
        public final static double THATA_KP = 4.4;
        public final static double THATA_KI = 0;
        public final static double THATA_KD = 0;

        // ------------2024--------------------
        public final static double SPEAKER_TARGET_X_BLUE = 0;
        public final static double SPEAKER_TAGET_X_RED = 16.54;
        public final static double SPEAKER_TARGET_Y = 5.55;

        public final static double START_OF_SPEAKER = 4.94 + 0.1;
        public final static double END_OF_SPEAKER = 6.13 - 0.1;

        public final static double ANGLE_PID_TOLORANCE = Math.toRadians(10);

        public final static double MAX_SHOOT_DISTANCE = 2.1;

        public final static double SHOOTING_IN_MOTION_TOLORANCE = 0.35;

        public final static double VISION_KP = 0; // TODO
        public final static double VISION_KI = 0; // TODO
        public final static double VISION_KD = 0; // TODO
        public final static double VISION_TOLORANCE = 0; // TODO

        public final static double SHOOTING_SPEED = 0.12;

        public final static double LOWER_SPEED = 0.4;

        public final static double PODUIM_LINE_RED = 13.390;
        public final static double PODUIM_LINE_BLUE = 3.282;

        public final static double MAX_LIMELIGHT_DIS = 2.5;
        
        public static double lowerSpeedFactor = LOWER_SPEED;
}
