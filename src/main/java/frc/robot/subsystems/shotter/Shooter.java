// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shotter;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.MAShuffleboard.pidControllerGainSupplier;
import com.ma5951.utils.subsystem.InternallyControlledSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class Shooter extends SubsystemBase implements InternallyControlledSubsystem {
  private static Shooter instance;

  private CANSparkMax master;
  private CANSparkMax slave;

  private RelativeEncoder encoder;
  private SparkPIDController pidController;
  private SimpleMotorFeedforward feedforward;

  private double setPoint;

  private MAShuffleboard board;
  private pidControllerGainSupplier pidGainSupplier;

  public Shooter() {
    master = new CANSparkMax(PortMap.Shooter.masterID, MotorType.kBrushless);
    slave = new CANSparkMax(PortMap.Shooter.slaveID, MotorType.kBrushless);

    master.setIdleMode(IdleMode.kBrake);
    slave.setIdleMode(IdleMode.kBrake);

    master.setInverted(false);
    slave.follow(master, true);

    encoder = master.getEncoder();
    encoder.setVelocityConversionFactor(ShooterConstants.VelocityConversionFactor);

    pidController = master.getPIDController();
    pidController.setFeedbackDevice(encoder);

    pidController.setP(ShooterConstants.kp);
    pidController.setP(ShooterConstants.ki);
    pidController.setP(ShooterConstants.kd);

    feedforward = new SimpleMotorFeedforward(0, ShooterConstants.kv);


    board = new MAShuffleboard("shotter");
    pidGainSupplier = board.getPidControllerGainSupplier(
      ShooterConstants.kp,
      ShooterConstants.ki,
      ShooterConstants.kd);
  }

  @Override
  public void setVoltage(double voltage) {
    master.set(voltage / 12);
  }

  @Override
  public boolean canMove() {
    return true;
  }

  @Override
  public void calculate(double setPoint) {
    pidController.setReference(setPoint, ControlType.kPosition, 0,
      feedforward.calculate(setPoint), ArbFFUnits.kPercentOut);
  }

  @Override
  public boolean atPoint() {
    return Math.abs(setPoint - encoder.getVelocity()) < ShooterConstants.tolorance; 
  }

  @Override
  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  @Override
  public void periodic() {
    pidController.setP(pidGainSupplier.getKP());
    pidController.setI(pidGainSupplier.getKI());
    pidController.setD(pidGainSupplier.getKD());

    board.addNum("v", encoder.getVelocity());
  }
}
