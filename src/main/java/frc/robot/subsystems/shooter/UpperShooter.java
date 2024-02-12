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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class UpperShooter extends SubsystemBase implements DefaultInternallyControlledSubsystem {
  private static UpperShooter instance;

  private final CANSparkMax motor;

  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;
  private final SimpleMotorFeedforward feedforward;

  private final DigitalInput sensor;

  private double setPoint = ShooterConstants.defaultVUp;

  public boolean changeToDefaultV = false;

  private final MAShuffleboard board;

  private UpperShooter() {
    motor = new CANSparkMax(PortMap.Shooter.upperID, MotorType.kBrushless);

    motor.restoreFactoryDefaults();

    motor.setIdleMode(IdleMode.kBrake);

    motor.setInverted(false);

    motor.setSmartCurrentLimit(40);

    motor.enableVoltageCompensation(12);

    encoder = motor.getEncoder();
    encoder.setVelocityConversionFactor(ShooterConstants.CONVERTION_FACTOR_UPPER);

    pidController = motor.getPIDController();
    pidController.setFeedbackDevice(encoder);

    pidController.setP(ShooterConstants.KP_UP);
    pidController.setI(ShooterConstants.KI_UP);
    pidController.setD(ShooterConstants.KD_UP);

    sensor = new DigitalInput(PortMap.Shooter.sensorID);

    feedforward = new SimpleMotorFeedforward(0, ShooterConstants.KV_UP);


    board = new MAShuffleboard("Upper shotter");

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
    return ShooterConstants.sample(
      SwerveDrivetrainSubsystem.getInstance().disFormSpeaker)[0] * ShooterConstants.V_FACTOR;
  }

  public static UpperShooter getInstance() {
    if (instance == null) {
      instance = new UpperShooter();
    }
    return instance;
  }

  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    board.addNum("v", getVelocity());

    board.addBoolean("sensor", isGamePiceInShooter());

    double poduimLine = DriverStation.getAlliance().get() == Alliance.Red ?
      SwerveConstants.PODUIM_LINE_RED : SwerveConstants.PODUIM_LINE_BLUE;
    double factor = DriverStation.getAlliance().get() == Alliance.Red ?
      -1 : 1;

    if (Intake.getInstance().isGamePieceInIntake() && !RobotContainer.isAmp
      && (SwerveDrivetrainSubsystem.getInstance().getPose().getX() * factor
       < poduimLine * factor || RobotContainer.driverController.L2().getAsBoolean())) {
        if (RobotContainer.ShootingLinkedToSpeaker) {
          ShooterConstants.defaultVUp = ShooterConstants.SPEAKER_UPPER_V;
          ShooterConstants.defaultVDown = ShooterConstants.SPEAKER_LOWER_V;
        } else {
          ShooterConstants.defaultVUp = ShooterConstants.
          sample(SwerveDrivetrainSubsystem.getInstance().disFormSpeaker)[0] * ShooterConstants.V_FACTOR;
          ShooterConstants.defaultVDown = ShooterConstants.
          sample(SwerveDrivetrainSubsystem.getInstance().disFormSpeaker)[1] * ShooterConstants.V_FACTOR;
        }
    } else {
      ShooterConstants.defaultVUp = 0;
      ShooterConstants.defaultVDown = 0;
    }

    if (changeToDefaultV && DriverStation.isTeleop()) {
      LowerShooter.getInstance().setSetPoint(ShooterConstants.defaultVUp);
      setSetPoint(ShooterConstants.defaultVDown);
    }

    board.addBoolean("atpoint", atPoint());

    board.addNum("set", setPoint);

    board.addNum("current", getCurrent());
  }
}
