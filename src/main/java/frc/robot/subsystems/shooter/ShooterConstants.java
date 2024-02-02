package frc.robot.subsystems.shooter;

public class ShooterConstants {

    public static final double CONVERTION_FACTOR_UPPER = 1.36;
    public static final double CONVERTION_FACTOR_LOWER = 1.33;
    public static final double TOLORANCE = 120;

    public static final double KP_UP = 0.0005;
    public static final double KI_UP = 0;
    public static final double KD_UP = 0;

    public static final double KP_LOW = 0.0005;
    public static final double KI_LOW = 0;
    public static final double KD_LOW = 0;
    
    public static final double KV_UP = 1.46e-4;
    public static final double KV_LOW = 1.7e-4;

    public static final double PODIUM_UPPER_V = 3675;
    public static final double PODUIM_LOWER_V = 1175;

    public static final double SPEAKER_UPPER_V = 1800;
    public static final double SPEAKER_LOWER_V = 3300;

    public static final double AMP_V_UPPER = 0;
    public static final double AMP_V_LOWER = 0.25;
    public static final double INTAKE_SOURCE_V = -0.4;

    public static final double V_FACTOR = 1.1;

    public static double defaultV = 0;

    // dis, up, down
    public static final double[][] shootingPoses = {
        {1.64, 1880, 1970},
        {1.78, 1780, 1810},
        {1.83, 1780, 1810},
        {1.9, 1780, 1810},
        {1.95, 1800, 1810},
        {1.983, 1800, 1810},
        {1.986, 1780, 1810},
        {2.04, 1780, 1810},
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
