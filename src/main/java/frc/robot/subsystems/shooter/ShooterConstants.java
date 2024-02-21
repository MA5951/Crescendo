package frc.robot.subsystems.shooter;

public class ShooterConstants {

    public static final double CONVERTION_FACTOR_UPPER = 1.36;
    public static final double CONVERTION_FACTOR_LOWER = 1.33;
    public static final double TOLORANCE = 92.2;
    public static final double TIME_AT_SETPOINT = 0.15;

    public static final double KP_UP = 0.00035;
    public static final double KI_UP = 0;
    public static final double KD_UP = 0.0000003;

    public static final double KP_LOW = 0.00055;
    public static final double KI_LOW = 0;
    public static final double KD_LOW = 0;
    
    public static final double KV_UP = 1.46e-4;
    public static final double KV_LOW = 1.7e-4;

    public static final double SPEAKER_UPPER_V = 1800;
    public static final double SPEAKER_LOWER_V = 3300;

    public static final double SPEAKER_UPPER_V_AUTO = 2200;
    public static final double SPEAKER_LOWER_V_AUTO= 3100;

    public static final double AMP_V_UPPER = 0;
    public static final double AMP_V_LOWER = 0.35;
    public static final double INTAKE_SOURCE_V = -0.4;

    public static final double V_FACTOR = 1.07;

    public static double defaultVUp = 0;
    public static double defaultVDown = 0;

    public static double getTolorance(double setPoint) {
        return Math.max(0.5952 * setPoint + 5.8986, TOLORANCE);
    }

    // dis, up, down
    public static final double[][] shootingPoses = {
        {1.55, 2100, 2180},
        {1.64, 1900, 1980},
        {1.78, 1760, 1790},
        {1.83, 1750, 1780},
        {1.9, 1750, 1780},
        {1.95, 1750, 1780}// ,
        // {1.983, 1740, 1760},
        // {1.986, 1720, 1720},
        // {2.04, 1960, 1450},
        // {2.27, 1960, 1450}
    };

    // up, down
    public static double[] sample(double disFromTarget) {
        final int dis = 0, up = 1, down = 2;
        double[] closestPose = new double[3];
        double minDis = Double.MAX_VALUE;
        int indexOfClosetPose = 0;
        for (int i = 0; i < shootingPoses.length; i++) {
            if (minDis > Math.abs(
                disFromTarget -
                shootingPoses[i][dis])) {
                closestPose = shootingPoses[i];
                minDis = Math.abs(
                    disFromTarget -
                    closestPose[dis]);
                indexOfClosetPose = i;
            }
        }
        if ((indexOfClosetPose == 0 && 
            disFromTarget < closestPose[dis])
            || (indexOfClosetPose == shootingPoses.length -1
            && disFromTarget > closestPose[dis])) {
                return new double[] {closestPose[up], closestPose[down]};
        }
        double[] secondCloset = 
            disFromTarget > closestPose[dis]
            ? shootingPoses[indexOfClosetPose + 1] :
            shootingPoses[indexOfClosetPose - 1];
        double[] smallerDisPose;
        double[] biggerDisPose;
        if (secondCloset[dis] < closestPose[dis]) {
            smallerDisPose = secondCloset;
            biggerDisPose = closestPose;
        } else {
            smallerDisPose = closestPose;
            biggerDisPose = secondCloset;
        }
        double t = 
            (disFromTarget
            - smallerDisPose[dis]) /
            (biggerDisPose[dis] - smallerDisPose[dis]);
        double upV = smallerDisPose[up] + (biggerDisPose[up] - smallerDisPose[up]) * t;
        double downV = smallerDisPose[down] + (biggerDisPose[down] - smallerDisPose[down]) * t;
        return new double[] {upV, downV};
    }
}
