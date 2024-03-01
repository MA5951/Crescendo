package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    public static final double KP = 1.5;
    public static final double KI = 0;
    public static final double KD = 0.004;
    public static final double TOLERANCE = 0.01;
    public static final double KP_CLIMB = 0.8;

    public static final double MIN_POSE = 0.01;
    public static final double MAX_POSE = 0.45;

    public static double DEFAULT_POSE = 0;;
    public static final double DEFAULT_POSE_STAGE = 0.17;
    public static final double DEFAULT_POSE_DEFANCE = 0.33;

    public static final double SHOOTING_POSE = 0.27; // 0.24;

    public static final double AMP_POSE = MAX_POSE;
    public static final double INTAKE_POSE = MIN_POSE;
    public static final double EJECT_POSE = MAX_POSE;
    public static final double SOURCE_POSE = MAX_POSE;
    public static final double CENTER_POSE = INTAKE_POSE + 0.05;

    public static final double CLIMB_POSE = 0.38;
    public static final double CLOSE_CLIMB_POSE = 0.058;

    public static final double WHEEL_RADIUS = 1.61671;

    public static final double GEAR = 10.9;

    public static final double POSITION_CONVERSION_FACTOR =
        ((2 * WHEEL_RADIUS * 
        Math.PI) / GEAR) / 100;

    public static final double CLOSING_POWER = -0.25;
    
    public static final double CURRENT_THRESHOLD = 20;
    public static final double TIME_WITH_CURRENT_JUMP = 0.1;
}