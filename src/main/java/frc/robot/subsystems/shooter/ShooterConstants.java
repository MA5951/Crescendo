package frc.robot.subsystems.shooter;

import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class ShooterConstants {

    public static final double CONVERTION_FACTOR_UPPER = 1.36;
    public static final double CONVERTION_FACTOR_LOWER = 1.33;
    public static final double TOLORANCE = 50;

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

    public static final double SPEAKER_UPPER_V = 3000;
    public static final double SPEAKER_LOWER_V = 4500;

    public static final double AMP_V_UPPER = 0;
    public static final double AMP_V_LOWER = 0.25;
    public static final double EJECT_V = 1000;
    public static final double INTAKE_SOURCE_V = -0.4;

    public static double defaultV = 0;

    // dis, up, down
    public static final double[][] shootingPoses = {
        {1.64, 1880, 1970},
        {1.78, 1780, 1810},
        {1.9, 2075, 2120},
        {1.98, 1780, 1810},
        {2.09, 2100, 1900},
        {2.1, 1780, 1810}
    };

    // up, down
    public static double[] semple() {
        final int dis = 0, up = 1, down = 2;
        double[] closestPose = new double[3];
        double minDis = Double.MAX_VALUE;
        int indexOfClosetPose = 0;
        for (int i = 0; i < shootingPoses.length; i++) {
            if (minDis > Math.abs(
                SwerveDrivetrainSubsystem.getInstance().disFormSpeaker -
                shootingPoses[i][dis])) {
                closestPose = shootingPoses[i];
                minDis = Math.abs(
                    SwerveDrivetrainSubsystem.getInstance().disFormSpeaker -
                    closestPose[dis]);
                indexOfClosetPose = i;
            }
        }
        if ((indexOfClosetPose == 0 && 
            SwerveDrivetrainSubsystem.getInstance().disFormSpeaker < closestPose[dis])
            || (indexOfClosetPose == shootingPoses.length -1
            && SwerveDrivetrainSubsystem.getInstance().disFormSpeaker > closestPose[dis])) {
                return new double[] {closestPose[up], closestPose[down]};
        }
        double[] secondCloset = 
            SwerveDrivetrainSubsystem.getInstance().disFormSpeaker > closestPose[dis]
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
            (SwerveDrivetrainSubsystem.getInstance().disFormSpeaker
            - smallerDisPose[dis]) /
            (biggerDisPose[dis] - smallerDisPose[dis]);
        double upV = smallerDisPose[up] + (biggerDisPose[up] - smallerDisPose[up]) * t;
        double downV = smallerDisPose[down] + (biggerDisPose[down] - smallerDisPose[down]) * t;
        return new double[] {upV, downV};
    }
}
