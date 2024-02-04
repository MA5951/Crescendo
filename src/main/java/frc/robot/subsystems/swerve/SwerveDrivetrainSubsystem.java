// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.RobotConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.RobotContainer;
import frc.robot.automations.AutoAutomations.ShootInMotion;

public class SwerveDrivetrainSubsystem extends SubsystemBase {

  private static SwerveDrivetrainSubsystem swerve;

  public final boolean isXReversed = true;
  public final boolean isYReversed = true;
  public final boolean isXYReversed = true;

  private double offsetAngle = 0;

  private double acc = 0;
  private double lastVelocity = 0;

  public double maxVelocity = SwerveConstants.MAX_VELOCITY;
  public double maxAngularVelocity = SwerveConstants.MAX_ANGULAR_VELOCITY;

  public double disFormSpeaker = 0;
  public double disFromSpeakerX = 0;

  public final MAShuffleboard board;

  private boolean canShoot;

  private final Translation2d frontLeftLocation = new Translation2d(
      SwerveConstants.WIDTH / 2,
      SwerveConstants.LENGTH / 2);
  private final Translation2d frontRightLocation = new Translation2d(
      -SwerveConstants.WIDTH / 2,
      SwerveConstants.LENGTH / 2);
  private final Translation2d rearLeftLocation = new Translation2d(
      SwerveConstants.WIDTH / 2,
      -SwerveConstants.LENGTH / 2);
  private final Translation2d rearRightLocation = new Translation2d(
      -SwerveConstants.WIDTH / 2,
      -SwerveConstants.LENGTH / 2);

  private final Pigeon2 gyro = new Pigeon2(PortMap.Swerve.Pigeon2ID, PortMap.CanBus.RioBus);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation,
    rearLeftLocation, rearRightLocation);

  private final static SwerveModule frontLeftModule = new SwerveModuleTalonFX(
      "frontLeftModule",
      PortMap.Swerve.leftFrontDriveID,
      PortMap.Swerve.leftFrontTurningID,
      PortMap.Swerve.leftFrontAbsoluteEncoder,
      SwerveConstants.FRONT_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
      SwerveConstants.FRONT_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED,
      SwerveConstants.FRONT_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED,
      SwerveConstants.FRONT_LEFT_MODULE_OFFSET_ENCODER,
      PortMap.CanBus.CANivoreBus);

  private final static SwerveModule frontRightModule = new SwerveModuleTalonFX(
      "frontRightModule",
      PortMap.Swerve.rightFrontDriveID,
      PortMap.Swerve.rightFrontTurningID,
      PortMap.Swerve.rightFrontAbsoluteEncoder,
      SwerveConstants.FRONT_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
      SwerveConstants.FRONT_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED,
      SwerveConstants.FRONT_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED,
      SwerveConstants.FRONT_RIGHT_MODULE_OFFSET_ENCODER,
      PortMap.CanBus.CANivoreBus);

  private final static SwerveModule rearLeftModule = new SwerveModuleTalonFX(
      "rearLeftModule",
      PortMap.Swerve.leftBackDriveID,
      PortMap.Swerve.leftBackTurningID,
      PortMap.Swerve.leftBackAbsoluteEncoder,
      SwerveConstants.REAR_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
      SwerveConstants.REAR_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED,
      SwerveConstants.REAR_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED,
      SwerveConstants.REAR_LEFT_MODULE_OFFSET_ENCODER,
      PortMap.CanBus.CANivoreBus);

  private final static SwerveModule rearRightModule = new SwerveModuleTalonFX(
      "rearRightModule",
      PortMap.Swerve.rightBackDriveID,
      PortMap.Swerve.rightBackTurningID,
      PortMap.Swerve.rightBackAbsoluteEncoder,
      SwerveConstants.REAR_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
      SwerveConstants.REAR_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED,
      SwerveConstants.REAR_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED,
      SwerveConstants.REAR_RIGHT_MODULE_OFFSET_ENCODER,
      PortMap.CanBus.CANivoreBus);

  private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(kinematics,
      new Rotation2d(0), getSwerveModulePositions(),
      new Pose2d(0, 0, new Rotation2d(0)));

  private final Field2d field = new Field2d();

  private static SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        rearLeftModule.getPosition(),
        rearRightModule.getPosition()
    };
  }

  private static SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        frontLeftModule.getState(),
        frontRightModule.getState(),
        rearLeftModule.getState(),
        rearRightModule.getState()
    };
  }

  /** Creates a new DrivetrainSubsystem. */
  private SwerveDrivetrainSubsystem() {

    resetGyro();

    this.board = new MAShuffleboard("swerve");

    SmartDashboard.putData("Field", field);

    BooleanSupplier flipPath = () -> {
      return DriverStation.getAlliance().get() == Alliance.Red;
    };

    AutoBuilder.configureHolonomic(
          this::getPose,
          this::resetOdometry,
          this::getRobotRelativeSpeeds,
          this::driveAuto,
          new HolonomicPathFollowerConfig(
            new PIDConstants(SwerveConstants.KP_TRANSLATION),
            new PIDConstants(SwerveConstants.THATA_KP,
              SwerveConstants.THATA_KI, SwerveConstants.THATA_KD),
            SwerveConstants.MAX_VELOCITY,
            SwerveConstants.RADIUS,
            new ReplanningConfig()
          ),
          flipPath,
          this
        );
  }

  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    rearLeftModule.resetEncoders();
    rearRightModule.resetEncoders();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
  }

  public void updateOffset() {
    offsetAngle = getFusedHeading();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double getAngularVelocity() {
    return this.kinematics.toChassisSpeeds(getSwerveModuleStates()).omegaRadiansPerSecond;
  }

  public double getRadialAcceleration() {
    return Math.pow(getAngularVelocity(), 2) * SwerveConstants.RADIUS;
  }

  public double getFusedHeading() {
    StatusSignal<Double> yaw = gyro.getYaw();
    yaw.refresh();
    return yaw.getValue();
  }

  public double getRoll() {
    StatusSignal<Double> roll = gyro.getRoll();
    roll.refresh();
    return roll.getValue();
  }

  public double getPitch() {
    StatusSignal<Double> pitch = gyro.getPitch();
    pitch.refresh();
    return pitch.getValue();
  }

  public double getVelocity() {
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    return Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) +
      Math.pow(speeds.vyMetersPerSecond, 2));
  }

  public double getAcceleration() {
    return acc;
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Math.toRadians(getFusedHeading()));
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public void FactorVelocityTo(double factor) {
    maxVelocity = SwerveConstants.MAX_VELOCITY * factor;
    maxAngularVelocity = SwerveConstants.MAX_ANGULAR_VELOCITY * factor;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public void stop() {
    frontLeftModule.stop();
    frontRightModule.stop();
    rearLeftModule.stop();
    rearRightModule.stop();
  }

  public void setModules(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY);
    frontLeftModule.setDesiredState(states[0]);
    rearLeftModule.setDesiredState(states[1]);
    frontRightModule.setDesiredState(states[2]);
    rearRightModule.setDesiredState(states[3]);
  }

  public void drive(double x, double y, double omega, boolean fieldRelative) {
    SwerveModuleState[] states = kinematics
        .toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega,
                new Rotation2d(
                  Math.toRadians((getFusedHeading() - offsetAngle))))
                : new ChassisSpeeds(x, y, omega));
    setModules(states);
  }

  public void driveAuto(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
      speeds.omegaRadiansPerSecond, false);
  }

  public Command getAutonomousPathCommand(String autoName) {
        return AutoBuilder.buildAuto(autoName);
  }

  public void setAccelerationLimit(double limit) {
    frontLeftModule.setAccelerationLimit(limit);
    frontRightModule.setAccelerationLimit(limit);
    rearLeftModule.setAccelerationLimit(limit);
    rearRightModule.setAccelerationLimit(limit);
  }

  public void setOffsetangle(double offset) {
    offsetAngle = offset;
  }

  public boolean canShoot() {
    return canShoot;
  }

  public static SwerveDrivetrainSubsystem getInstance() {
    if (swerve == null) {
      swerve = new SwerveDrivetrainSubsystem();
    }
    return swerve;
  }

  @Override
  public void periodic() {
    acc = (frontLeftModule.getDriveVelocity() - lastVelocity) / RobotConstants.KDELTA_TIME;
    odometry.update(getRotation2d(), getSwerveModulePositions());

    lastVelocity = frontLeftModule.getDriveVelocity();

    field.setRobotPose(getPose());

    board.addNum("yaw", getFusedHeading());
    board.addNum("roll", getRoll());
    board.addNum("pitch", getPitch());

    board.addNum("yaw pose", getPose().getRotation().getDegrees());

    board.addNum("flV", frontLeftModule.getDriveVelocity());

    board.addBoolean("can shoot", canShoot);

    double ySpeaker = SwerveConstants.SPEAKER_TARGET_Y;
    double xSpeaker =  DriverStation.getAlliance().get() == Alliance.Blue ? 
      SwerveConstants.SPEAKER_TARGET_X_BLUE : SwerveConstants.SPEAKER_TAGET_X_RED;

    canShoot = getPose().getTranslation()
      .getDistance(new Translation2d(xSpeaker, ySpeaker)) < 
        SwerveConstants.MAX_SHOOT_DISTANCE;

    board.addBoolean("can shoot", canShoot);

    disFromSpeakerX = new Translation2d(
      SwerveDrivetrainSubsystem.getInstance().getPose().getX(),
        0).getDistance(new Translation2d(
        DriverStation.getAlliance().get() == Alliance.Blue ?
        SwerveConstants.SPEAKER_TARGET_X_BLUE : 
        SwerveConstants.SPEAKER_TAGET_X_RED, 0));

      disFormSpeaker = new Translation2d(xSpeaker, ySpeaker).getDistance(
        getPose().getTranslation()
      );

    if (RobotContainer.APRILTAGS_LIMELIGHT.hasTarget()
      && RobotContainer.APRILTAGS_LIMELIGHT.getTagId() != -1
      && !DriverStation.isAutonomous()) {
     // && RobotContainer.APRILTAGS_LIMELIGHT.getA() > SwerveConstants.MAX_LIMELIGHT_DIS) {
      Pose2d estPose = RobotContainer.APRILTAGS_LIMELIGHT.getEstPose();
      resetOdometry(estPose);
    }

    if (ShootInMotion.isRunning) {
      SwerveConstants.lowerSpeedFactor = SwerveConstants.LOWER_SPEED * 
        SwerveConstants.SHOOTING_SPEED;
    } else {
      SwerveConstants.lowerSpeedFactor = SwerveConstants.LOWER_SPEED;
    }

  }
}