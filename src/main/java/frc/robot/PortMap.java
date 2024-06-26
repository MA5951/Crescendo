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

    public static final int leftBackAbsoluteEncoder = 21;
    public static final int leftBackDriveID = 4;
    public static final int leftBackTurningID = 9;

    public static final int rightFrontAbsoluteEncoder = 23;
    public static final int rightFrontDriveID = 7;
    public static final int rightFrontTurningID = 6;

    public static final int rightBackAbsoluteEncoder = 24;
    public static final int rightBackDriveID = 3;
    public static final int rightBackTurningID = 2;

    public static final int Pigeon2ID = 12;
  }

  public static class Shooter {
    public static final int upperID = 14;
    public static final int lowerID = 18;

    public static final int sensorID = 7;
  }

  public static class Intake {
    public static final int masterID = 17;
    public static final int slaveID = 13;

    public static final int sensor1ID = 8; 
    public static final int sensor2ID = 9; 
  }

  public static class Elevator {
    public static final int masterID = 28;
    public static final int slaveID = 30;
  }

  public static class LED {
    public static final int ledPort = 9;
  }
}