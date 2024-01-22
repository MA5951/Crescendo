package frc.robot.subsystems.intake;


import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.subsystem.MotorSubsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.subsystems.shooter.LowerShooter;
import frc.robot.subsystems.shooter.UpperShooter;

import com.revrobotics.CANSparkMax;

public class Intake extends SubsystemBase implements MotorSubsystem{
  private static Intake intake;
  
  private DigitalInput upSensor;
  private DigitalInput downSensor;

  private CANSparkMax master;
  private CANSparkMax slave;

  private MAShuffleboard board;

  private Intake() {
    master = new CANSparkMax(PortMap.Intake.masterID, MotorType.kBrushless);
    slave = new CANSparkMax(PortMap.Intake.slaveID, MotorType.kBrushless);

    upSensor = new DigitalInput(PortMap.Intake.sensor1ID);
    downSensor = new DigitalInput(PortMap.Intake.sensor2ID);
    board = new MAShuffleboard("Intake");

    master.setIdleMode(IdleMode.kBrake);
    master.setInverted(false);
    slave.setIdleMode(IdleMode.kBrake);
    slave.follow(master, false);
  
  }

  public boolean isGamePieceInIntake(){
    return !upSensor.get() || !downSensor.get();
  }

  @Override
  public boolean canMove() {
      return !isGamePieceInIntake() 
        || (LowerShooter.getInstance().atPoint() && 
        UpperShooter.getInstance().atPoint()) || getPower() < 0;
  }

  @Override
  public void setVoltage(double voltage) {
    master.set(voltage / 12);
  }

  public double getPower() {
    return master.get();
  }

  public static Intake getInstance() {
  if (intake == null) {
      intake = new Intake();  
    }
  return intake;
  }

  @Override
  public void periodic() {
    board.addBoolean("Sensor down", !downSensor.get());
    board.addBoolean("Sensor up", !upSensor.get());

    board.addNum("current", master.getOutputCurrent());
  }
}
