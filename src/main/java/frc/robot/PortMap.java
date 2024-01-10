package frc.robot;

public class PortMap {

  public static class CanBus {
    public static final String CANivoreBus = "Swerve";
    public static final String RioBus = "rio";
  }

  public static class Controllers {
    public static final int driveID = 0;
    public static final int operatorID = 1;    
  }

  public static class Swerve {
    public static final int leftFrontAbsoluteEncoder = 22;
    public static final int leftFrontDriveID = 8;
    public static final int leftFrontTurningID = 5;

    public static final int leftBackAbsoluteEncoder = 23;
    public static final int leftBackDriveID = 7;
    public static final int leftBackTurningID = 6;

    public static final int rightFrontAbsoluteEncoder = 21;
    public static final int rightFrontDriveID = 4;
    public static final int rightFrontTurningID = 9;

    public static final int rightBackAbsoluteEncoder = 24;
    public static final int rightBackDriveID = 2;
    public static final int rightBackTurningID = 3;

    public static final int Pigeon2ID = 12; // TODO
  }

  public static class Intake {
    public static final int masterID = 13; //TODO
    public static final int slvaeID = 14; //TODO

    public static final int sensorID = 0; //TODO
  }
}