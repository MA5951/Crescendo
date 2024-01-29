package frc.robot.subsystems.shooter;

public class ShooterConstants {

    public static final double CONVERTION_FACTOR_UPPER = 1.36;
    public static final double CONVERTION_FACTOR_LOWER = 1.33;
    public static final double TOLORANCE = 170;

    public static final double KP_UP = 0.0003;
    public static final double KI_UP = 0;
    public static final double KD_UP = 0;

    public static final double KP_LOW = 0.0007;
    public static final double KI_LOW = 0;
    public static final double KD_LOW = 0;
    
    public static final double KV_UP = 1.46e-4;
    public static final double KV_LOW = 1.7e-4;

    public static final double PODIUM_UPPER_V = 3675;
    public static final double PODUIM_LOWER_V = 1175;

    public static final double SPEAKER_UPPER_V = 3000;
    public static final double SPEAKER_LOWER_V = 4500;

    public static final double AMP_V_UPPER = 0;
    public static final double AMP_V_LOWER = 0.25;
    public static final double EJECT_V = 1000;
    public static final double INTAKE_SOURCE_V = -0.4;

    public static double defaultV = 0;
}
