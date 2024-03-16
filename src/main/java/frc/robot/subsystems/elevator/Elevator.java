package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.subsystem.DefaultInternallyControlledSubsystem;

import frc.robot.PortMap;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase implements DefaultInternallyControlledSubsystem{

    private static Elevator elevator;

    private final TalonFX master;
    private final TalonFX slave;

    private final PositionVoltage PoseSetter;

    private final StatusSignal<Double> pose;
    private double poseOffset = 0;

    private double setPoint = 0;

    private final MAShuffleboard board;

    private boolean disabledElevator = false;

    private Elevator() {
      master = new TalonFX(PortMap.Elevator.masterID, PortMap.CanBus.CANivoreBus);
      slave = new TalonFX(PortMap.Elevator.slaveID, PortMap.CanBus.CANivoreBus);
 
      configMotors();

      PoseSetter = new PositionVoltage(0);

      pose = master.getPosition();

      ElevatorConstants.DEFAULT_POSE = ElevatorConstants.DEFAULT_POSE_STAGE;

      resetPose(0);

      board = new MAShuffleboard("Elevator");

      board.createBoutton("Disable Elevator", new InstantCommand(() -> 
        disabledElevator = !disabledElevator));

    }

    public void configMotors() {
      TalonFXConfiguration configuration = new TalonFXConfiguration();

      configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

      configuration.ClosedLoopGeneral.ContinuousWrap = false;

      configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      configuration.CurrentLimits.StatorCurrentLimitEnable = true;
      configuration.CurrentLimits.StatorCurrentLimit = 25;

      configuration.Slot0.kP = ElevatorConstants.KP;
      configuration.Slot0.kI = ElevatorConstants.KI;
      configuration.Slot0.kD = ElevatorConstants.KD;

      master.getConfigurator().apply(configuration);

      slave.getConfigurator().apply(configuration);

      slave.setControl(new Follower(PortMap.Elevator.masterID, true));
    }

    public void configMotorsForClimb() {
      TalonFXConfiguration configuration = new TalonFXConfiguration();

      configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

      configuration.ClosedLoopGeneral.ContinuousWrap = false;

      configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      configuration.CurrentLimits.StatorCurrentLimitEnable = true;
      configuration.CurrentLimits.StatorCurrentLimit = 25;

      configuration.Slot0.kP = 1;
      configuration.Slot0.kI = ElevatorConstants.KI;
      configuration.Slot0.kD = ElevatorConstants.KD;

      master.getConfigurator().apply(configuration);

      slave.getConfigurator().apply(configuration);

      slave.setControl(new Follower(PortMap.Elevator.masterID, true));
    }

    public double getCurrent() {
        return master.getStatorCurrent().refresh().getValueAsDouble();
    }

    @Override
    public void calculate(double setPoint) {
        master.setControl(PoseSetter.withPosition(
          (setPoint + poseOffset) / ElevatorConstants.POSITION_CONVERSION_FACTOR)
          .withSlot(0));
    }

    @Override
    public boolean atPoint() {
        return Math.abs(getPosition() - getSetPoint()) <= ElevatorConstants.TOLERANCE
         || disabledElevator;
    }

    @Override
    public void setSetPoint(double setPoint) {
        if (setPoint < ElevatorConstants.MIN_POSE) {
            this.setPoint = ElevatorConstants.MIN_POSE;
        } else if (setPoint > ElevatorConstants.MAX_POSE) {
            this.setPoint = ElevatorConstants.MAX_POSE;
        } else {
            this.setPoint = setPoint;
        }
    }

    @Override
    public double getSetPoint() {
        return setPoint;
    }
    
    public double getPosition() {
        pose.refresh();
        return (pose.getValue() * ElevatorConstants.POSITION_CONVERSION_FACTOR - poseOffset);
    }

    public void resetPose(double pose) {
      this.pose.refresh();
      poseOffset = 
        this.pose.getValueAsDouble() * ElevatorConstants.POSITION_CONVERSION_FACTOR + pose;
    }

    @Override
    public void setVoltage(double voltage) {
        master.set(voltage / 12);
    }

    @Override
    public boolean canMove() {
        return setPoint >= ElevatorConstants.MIN_POSE &&
         setPoint <= ElevatorConstants.MAX_POSE && !disabledElevator;
    }

    public void toggleDeafultPose() {
        if (ElevatorConstants.DEFAULT_POSE == ElevatorConstants.DEFAULT_POSE_STAGE) {
            ElevatorConstants.DEFAULT_POSE = ElevatorConstants.DEFAULT_POSE_DEFANCE;
        } else {
            ElevatorConstants.DEFAULT_POSE = ElevatorConstants.DEFAULT_POSE_STAGE;
        }
    }

    public static Elevator getInstance() {
        if (elevator == null) {
          elevator = new Elevator();
        }
        return elevator;
    }

    @Override
    public void periodic() {
        board.addBoolean("at point", atPoint());

        board.addBoolean("Disable elevator", disabledElevator);
        board.addNum("setPoint", getSetPoint());
        board.addNum("pose", getPosition());



        if (RobotContainer.driverController.getHID().getL2Button()) {
            setPoint = ElevatorConstants.SHOOTING_POSE;
        }
    }
}
