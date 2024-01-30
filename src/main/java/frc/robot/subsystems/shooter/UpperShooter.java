// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.subsystem.DefaultInternallyControlledSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class UpperShooter extends SubsystemBase implements DefaultInternallyControlledSubsystem {
  private static UpperShooter instance;

  private final CANSparkMax motor;

  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;
  private final SimpleMotorFeedforward feedforward;

  private final DigitalInput sensor;

  private double setPoint = ShooterConstants.defaultV;

  private final MAShuffleboard board;

  private UpperShooter() {
    motor = new CANSparkMax(PortMap.Shooter.upperID, MotorType.kBrushless);

    motor.restoreFactoryDefaults();

    motor.setIdleMode(IdleMode.kBrake);

    motor.setInverted(false);

    encoder = motor.getEncoder();
    encoder.setVelocityConversionFactor(ShooterConstants.CONVERTION_FACTOR_UPPER);
    encoder.setPositionConversionFactor(ShooterConstants.CONVERTION_FACTOR_UPPER);

    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);

    pidController.setP(ShooterConstants.KP_UP);
    pidController.setI(ShooterConstants.KI_UP);
    pidController.setD(ShooterConstants.KD_UP);

    sensor = new DigitalInput(PortMap.Shooter.sensorID);

    feedforward = new SimpleMotorFeedforward(0, ShooterConstants.KV_UP);


    board = new MAShuffleboard("Upper shotter");

    board.addNum("setPonit poduim", ShooterConstants.PODIUM_UPPER_V);
  }

  public void chengeIDLmode(IdleMode mode) {
    motor.setIdleMode(mode);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.set(voltage / 12);
  }

  public boolean isGamePiceInShooter() {
    return !sensor.get();
  }

  @Override
  public boolean canMove() {
    return true;
  }

  @Override
  public void calculate(double setPoint) {
    pidController.setReference(setPoint, ControlType.kVelocity, 0,
      feedforward.calculate(setPoint), ArbFFUnits.kPercentOut);
  }

  @Override
  public boolean atPoint() {
    return Math.abs(getSetPoint() - getVelocity()) < ShooterConstants.TOLORANCE; 
  }

  public double getVelocity(){
    return encoder.getVelocity();
  }

  @Override
  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }

  public void setSetPoint(Supplier<Double> setPoint) {
    this.setPoint = setPoint.get();
  }

  @Override
  public double getSetPoint() {
    return setPoint;
  }
  
  public double getDistance() {
    return encoder.getPosition();
  }

  public void resetEncoder(double pose) {
    encoder.setPosition(pose);
  }

  public double getVelocityForShooting() {
    double dis = SwerveDrivetrainSubsystem.getInstance().disFormSpeaker;
    double factor = 1;
    return ShooterConstants.semple()[0] * factor;
  }

  public double getPoduim() {
    return board.getNum("setPonit poduim");
  }

  public static UpperShooter getInstance() {
    if (instance == null) {
      instance = new UpperShooter();
    }
    return instance;
  }

  @Override
  public void periodic() {
    board.addNum("v", getVelocity());

    board.addBoolean("sensor", isGamePiceInShooter());

    if (Intake.getInstance().isGamePieceInIntake()) {
      ShooterConstants.defaultV = 1500;
    } else {
      ShooterConstants.defaultV = 0;
    }

    board.addBoolean("atpoint", atPoint());

    board.addNum("set", setPoint);
  }
}
