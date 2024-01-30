package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    public static final double KP = 42;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double TOLERANCE = 0.01;

    public static final double MIN_POSE = 0.01;
    public static final double MAX_POSE = 0.52;

    public static final double DEFAULT_POSE = 0.24;

    public static final double AMP_POSE = MAX_POSE;
    public static final double SHOOTING_POSE_PODUIM = DEFAULT_POSE;
    public static final double INTAKE_POSE = MIN_POSE;
    public static final double EJECT_POSE = MAX_POSE;
    public static final double SOURCE_POSE = MAX_POSE;

    public static final double CLIMB_POSE = MAX_POSE;
    public static final double CLOSE_CLIMB_POSE = 0.37;

    public static final double WHEEL_RADIUS = 1.61671;

    public static final double GEAR = 11.9;

    public static final double POSITION_CONVERSION_FACTOR =
        ((2 * WHEEL_RADIUS * Math.PI) / GEAR) / 100;

    public static final double ABS_POSITION_CONVERTION_FACTOR = 1;

    public static final double CLOSING_POWER = -0.3;
    
    public static final double CURRENT_THRESHOLD = 30;
    public static final double TIME_WITH_CURRENT_JUMP = 0.1;
}