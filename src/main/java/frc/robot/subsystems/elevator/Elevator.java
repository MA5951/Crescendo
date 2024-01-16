package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.MAShuffleboard.pidControllerGainSupplier;
import com.ma5951.utils.subsystem.DefaultInternallyControlledSubsystem;

import frc.robot.PortMap;

public class Elevator extends SubsystemBase implements DefaultInternallyControlledSubsystem{

    private static Elevator elevator;

    // TODO to add absolute encoder

    private CANSparkMax master;
    private CANSparkMax slave;
    private RelativeEncoder encoder;

    private SparkPIDController pidController;
    private double setPoint = 0;

    private pidControllerGainSupplier pidGainSupplier;

    private MAShuffleboard board;


    private Elevator() {
      master = new CANSparkMax(PortMap.Elevator.masterID, MotorType.kBrushless);
      slave = new CANSparkMax(PortMap.Elevator.slaveID, MotorType.kBrushless);

      master.setIdleMode(IdleMode.kBrake);
      slave.setIdleMode(IdleMode.kBrake);

      encoder = master.getEncoder();

      encoder.setPositionConversionFactor(ElevatorConstants.positionConversionFactor);

      resetPose(0);

      pidController = master.getPIDController();
      pidController.setFeedbackDevice(encoder);
      pidController.setP(ElevatorConstants.kP);
      pidController.setI(ElevatorConstants.kI);
      pidController.setD(ElevatorConstants.kD);


      slave.follow(master, true);

      board = new MAShuffleboard("Elevator");
      pidGainSupplier = board.getPidControllerGainSupplier(
        "position",
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD);
    }

    
    @Override
    public void calculate(double setPoint) {
        pidController.setReference(setPoint, ControlType.kPosition);
    }

    @Override
    public boolean atPoint() {
        return Math.abs(getPosition() - setPoint) <= ElevatorConstants.tolerance;
    }

    @Override
    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }

    @Override
    public double getSetPoint() {
        return setPoint;
    }
    
    public double getPosition() {
        return encoder.getPosition();
    }

    public void resetPose(double pose) {
        encoder.setPosition(pose);
    }

    @Override
    public void setVoltage(double voltage) {
        master.set(voltage / 12);
    }

    @Override
    public boolean canMove() {
        return setPoint >= ElevatorConstants.minPose &&
         setPoint <= ElevatorConstants.maxPose;
    }

    public static Elevator getInstance() {
        if (elevator == null) {
          elevator = new Elevator();
        }
        return elevator;
    }

    @Override
    public void periodic() {
        board.addNum("pose", getPosition());
        board.addNum("setPoint", getSetPoint());

        pidController.setP(pidGainSupplier.getKP());
        pidController.setI(pidGainSupplier.getKI());
        pidController.setD(pidGainSupplier.getKD());
    }
}
