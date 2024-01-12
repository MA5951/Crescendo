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
  private CANSparkMax motor;

  private MAShuffleboard board;

  private Intake() {
    motor = new CANSparkMax(PortMap.Intake.motorID, MotorType.kBrushless);

    sensor1 = new DigitalInput(PortMap.Intake.sensor1ID);
    sensor2 = new DigitalInput(PortMap.Intake.sensor2ID);
    board = new MAShuffleboard("Intake");

    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(false);
  
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
      motor.set(voltage / 12);
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
