package frc.robot.subsystems.intake;


import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.subsystem.MotorSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import com.revrobotics.CANSparkMax;

public class Intake extends SubsystemBase implements MotorSubsystem{
  private static Intake intake;
  
  private DigitalInput upSensor;
  private DigitalInput downSensor2;

  private CANSparkMax master;
  private CANSparkMax slave;

  private MAShuffleboard board;

  private Intake() {
    master = new CANSparkMax(PortMap.Intake.masterID, MotorType.kBrushless);
    slave = new CANSparkMax(PortMap.Intake.slaveID, MotorType.kBrushless);

    sensor1 = new DigitalInput(PortMap.Intake.sensor1ID);
    sensor2 = new DigitalInput(PortMap.Intake.sensor2ID);
    board = new MAShuffleboard("Intake");

    master.setIdleMode(IdleMode.kBrake);
    master.setInverted(false);
    slave.setIdleMode(IdleMode.kBrake);
    slave.follow(master, true);
  
  }

  public boolean isGamePieceInIntake(){
    return !sensor1.get() || !sensor2.get();
  }

  @Override
  public boolean canMove() {
      return true;
  }

  @Override
  public void setVoltage(double voltage) {
      master.set(voltage / 12);
  }

  public static Intake getInstance() {
  if (intake == null) {
      intake = new Intake();  
    }
  return intake;
  }

  @Override
  public void periodic() {
    board.addBoolean("Sensor 1", !sensor1.get());
    board.addBoolean("Sensor 2", !sensor2.get());
  }
}
