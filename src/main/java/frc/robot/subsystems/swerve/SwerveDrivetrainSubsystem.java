// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.RobotConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class SwerveDrivetrainSubsystem extends SubsystemBase {

  private static SwerveDrivetrainSubsystem swerve;

  public boolean isXReversed = true;
  public boolean isYReversed = false;
  public boolean isXYReversed = true;

  private double offsetAngle = 0;

  private double acc = 0;
  private double lastVelocity = 0;

  public double maxVelocity = SwerveConstants.MAX_VELOCITY;
  public double maxAngularVelocity = SwerveConstants.MAX_ANGULAR_VELOCITY;

  private ProfiledPIDController thetaProfiledPID;

  private static final String theta_KP = "theta_KP";
  private static final String theta_KI = "theta_KI";
  private static final String theta_KD = "theta_KD";

  private static final String profiled_theta_KP = "Profiled_theta_KP";
  private static final String profiled_theta_KI = "Profiled_theta_KI";
  private static final String profiled_theta_KD = "Profiled_theta_KD";

  public final MAShuffleboard board;

  private final Translation2d frontLeftLocation = new Translation2d(
      -SwerveConstants.WIDTH / 2,
      SwerveConstants.LENGTH / 2);
  private final Translation2d frontRightLocation = new Translation2d(
      SwerveConstants.WIDTH / 2,
      SwerveConstants.LENGTH / 2);
  private final Translation2d rearLeftLocation = new Translation2d(
      -SwerveConstants.WIDTH / 2,
      -SwerveConstants.LENGTH / 2);
  private final Translation2d rearRightLocation = new Translation2d(
      SwerveConstants.WIDTH / 2,
      -SwerveConstants.LENGTH / 2);

  private final Pigeon2 gyro = new Pigeon2(PortMap.Swerve.Pigeon2ID, PortMap.CanBus.CANivoreBus);

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
        rearLeftModule.getPosition(),
        frontLeftModule.getPosition(),
        rearRightModule.getPosition(),
        frontRightModule.getPosition()
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

    board.addNum(theta_KP, SwerveConstants.THATA_KP);
    board.addNum(theta_KI, SwerveConstants.THATA_KI);
    board.addNum(theta_KD, SwerveConstants.THATA_KD);

    board.addNum(profiled_theta_KP, SwerveConstants.PROFILED_THATA_KP);
    board.addNum(profiled_theta_KI, SwerveConstants.PROFILED_THATA_KI);
    board.addNum(profiled_theta_KD, SwerveConstants.PROFILED_THATA_KD);

    thetaProfiledPID = new ProfiledPIDController(
        board.getNum(profiled_theta_KP), board.getNum(profiled_theta_KI),
        board.getNum(profiled_theta_KD),
        new TrapezoidProfile.Constraints(SwerveConstants.MAX_ANGULAR_VELOCITY,
            SwerveConstants.MAX_ANGULAR_ACCELERATION));

    thetaProfiledPID.enableContinuousInput(0, 2 * Math.PI);

    SmartDashboard.putData("Field", field);

    // AutoBuilder.configureHolonomic(
    //       this::getPose,
    //       this::resetOdometry,
    //       this::getRobotRelativeSpeeds,
    //       this::driveAuto,
    //       new HolonomicPathFollowerConfig(
    //         new PIDConstants(SwerveConstants.KP_TRANSLATION),
    //         new PIDConstants(SwerveConstants.THATA_KP,
    //           SwerveConstants.THATA_KI, SwerveConstants.THATA_KD),
    //         SwerveConstants.MAX_VELOCITY,
    //         SwerveConstants.RADIUS,
    //         new ReplanningConfig()
    //       ),
    //       this
    //     );
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
    return -yaw.getValue();
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
    return frontLeftModule.getDriveVelocity();
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
                new Rotation2d(Math.toRadians((getFusedHeading() - offsetAngle))))
                : new ChassisSpeeds(x, y, omega));
    setModules(states);
  }

  public void driveAuto(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
      speeds.omegaRadiansPerSecond, false);
  }

//   public Command getAutonomousPathCommand(String autoName) {
//         return AutoBuilder.buildAuto(autoName);
//   }

  public void setAccelerationLimit(double limit) {
    frontLeftModule.setAccelerationLimit(limit);
    frontRightModule.setAccelerationLimit(limit);
    rearLeftModule.setAccelerationLimit(limit);
    rearRightModule.setAccelerationLimit(limit);
  }

  public void setOffsetangle(double offset) {
    offsetAngle = offset;
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

    board.addNum("afl", frontLeftModule.getAbsoluteEncoderPosition());
    board.addNum("afr", frontRightModule.getAbsoluteEncoderPosition());
    board.addNum("arl", rearLeftModule.getAbsoluteEncoderPosition());
    board.addNum("arr", rearRightModule.getAbsoluteEncoderPosition());

    board.addNum("fl angle", frontLeftModule.getTurningPosition());
    board.addNum("fr angle", frontRightModule.getTurningPosition());
    board.addNum("rl angle", rearLeftModule.getTurningPosition());
    board.addNum("rr angle", rearRightModule.getTurningPosition());

    board.addNum("vfl angle", frontLeftModule.getDriveVelocity());
    board.addNum("vfr angle", frontRightModule.getDriveVelocity());
    board.addNum("vrl angle", rearLeftModule.getDriveVelocity());
    board.addNum("vrr angle", rearRightModule.getDriveVelocity());
  }
}