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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class LowerShooter extends SubsystemBase implements DefaultInternallyControlledSubsystem {
  private static LowerShooter instance;

  private final CANSparkMax motor;

  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;
  private final SimpleMotorFeedforward feedforward;

  private double setPoint = ShooterConstants.defaultVDown;

  private final MAShuffleboard board;

  private LowerShooter() {
    motor = new CANSparkMax(PortMap.Shooter.lowerID, MotorType.kBrushless);

    motor.restoreFactoryDefaults();

    motor.setIdleMode(IdleMode.kBrake);

    motor.setInverted(true);

    motor.setSmartCurrentLimit(40);

    motor.enableVoltageCompensation(12);

    encoder = motor.getEncoder();
    encoder.setVelocityConversionFactor(ShooterConstants.CONVERTION_FACTOR_LOWER);
    encoder.setPositionConversionFactor(ShooterConstants.CONVERTION_FACTOR_LOWER);

    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);

    pidController.setP(ShooterConstants.KP_LOW);
    pidController.setI(ShooterConstants.KI_LOW);
    pidController.setD(ShooterConstants.KD_LOW);

    feedforward = new SimpleMotorFeedforward(0, ShooterConstants.KV_LOW);


    board = new MAShuffleboard("Lower shotter");
  }

  public void chengeIDLmode(IdleMode mode) {
    motor.setIdleMode(mode);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.set(voltage / 12);
  }

  @Override
  public boolean canMove() {
    return UpperShooter.getInstance().canMove();
  }

  @Override
  public void calculate(double setPoint) {
    pidController.setReference(setPoint, ControlType.kVelocity, 0,
      feedforward.calculate(setPoint), ArbFFUnits.kPercentOut);
  }

  @Override
  public boolean atPoint() {
    return Math.abs(getSetPoint() - getVelocity()) < ShooterConstants.getTolorance(getSetPoint()); 
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
    return ShooterConstants.sample(
      SwerveDrivetrainSubsystem.getInstance().disFormSpeaker)[1] * ShooterConstants.V_FACTOR;
  }

  public static LowerShooter getInstance() {
    if (instance == null) {
      instance = new LowerShooter();
    }
    return instance;
  }

  @Override
  public void periodic() {
    board.addNum("v", getVelocity());

    board.addBoolean("atpoint", atPoint());

    board.addNum("set", setPoint);

    board.addNum("current", motor.getOutputCurrent());
  }
}
