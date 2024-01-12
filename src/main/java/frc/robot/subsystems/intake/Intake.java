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
  
  private DigitalInput sensor1;
  private DigitalInput sensor2; 
  private CANSparkMax master;
  private CANSparkMax slave;


  private MAShuffleboard board;

  private Intake() {
    master = new CANSparkMax(PortMap.Intake.masterID, MotorType.kBrushless);
    slave = new CANSparkMax(PortMap.Intake.slvaeID, MotorType.kBrushless);

    sensor1 = new DigitalInput(PortMap.Intake.sensor1ID);
    sensor2 = new DigitalInput(PortMap.Intake.sensor2ID);
    board = new MAShuffleboard("Intake");

    master.setIdleMode(IdleMode.kBrake);
    slave.setIdleMode(IdleMode.kBrake);
    master.setInverted(false);
    slave.follow(master, true);

  
  }

  public boolean getSensor1() {
    return sensor1.get();
  }

  public boolean getSensor2() {
    return sensor2.get();
  }

  public boolean isGamePiece(){
    return getSensor1() || getSensor2();
  }

  public double getIntakeCurrent() {
    return (master.getOutputCurrent() + slave.getOutputCurrent()) / 2;
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
    board.addNum("Avrage current", getIntakeCurrent());
    board.addBoolean("High Sensor", getSensor1());
    board.addBoolean("Low Sensor", getSensor2());
    board.addBoolean("Is Game Piece", isGamePiece());
  }


}
