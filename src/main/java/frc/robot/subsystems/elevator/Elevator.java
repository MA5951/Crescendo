package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.MAShuffleboard.pidControllerGainSupplier;
import com.ma5951.utils.subsystem.DefaultInternallyControlledSubsystem;

import frc.robot.PortMap;
import frc.robot.Robot;

public class Elevator extends SubsystemBase implements DefaultInternallyControlledSubsystem{

    private static Elevator elevator;

    private AnalogEncoder absEncoder;

    private CANSparkMax master;
    private CANSparkMax slave1;
    private CANSparkMax slave2;
    private RelativeEncoder encoder;

    private SparkPIDController pidController;
    private double setPoint = 0;

    private pidControllerGainSupplier pidGainSupplier;

    private MAShuffleboard board;


    private Elevator() {
      master = new CANSparkMax(PortMap.Elevator.masterID, MotorType.kBrushless);
      slave1 = new CANSparkMax(PortMap.Elevator.slave1ID, MotorType.kBrushless);
      slave2 = new CANSparkMax(PortMap.Elevator.slave2ID, MotorType.kBrushless);
      slave1.restoreFactoryDefaults();
      slave2.restoreFactoryDefaults();
      master.restoreFactoryDefaults();

      absEncoder = new AnalogEncoder(PortMap.Elevator.absEncoderID);

      master.setIdleMode(IdleMode.kCoast);
      slave1.setIdleMode(IdleMode.kCoast);
      slave2.setIdleMode(IdleMode.kCoast);
      slave1.follow(master, true);
      slave2.follow(master, true);

      encoder = master.getEncoder();

      encoder.setPositionConversionFactor(ElevatorConstants.positionConversionFactor);

      resetPose(absEncoder.getAbsolutePosition() * 
        ElevatorConstants.absPositionConversionFactor - ElevatorConstants.encoderOffset);

      resetPose(0);

      pidController = master.getPIDController();
      pidController.setFeedbackDevice(encoder);
      pidController.setP(ElevatorConstants.kP);
      pidController.setI(ElevatorConstants.kI);
      pidController.setD(ElevatorConstants.kD);


    //   slave1.follow(master, true);
    //   slave2.follow(master, true);

      board = new MAShuffleboard("Elevator");
      pidGainSupplier = board.getPidControllerGainSupplier(
        "position",
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD);
    }

    public double getCurrent() {
        return master.getOutputCurrent();
    }

    
    @Override
    public void calculate(double setPoint) {
        pidController.setReference(setPoint, ControlType.kPosition);
    }

    @Override
    public boolean atPoint() {
        return Math.abs(getPosition() - getSetPoint()) <= ElevatorConstants.tolerance;
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
        board.addNum("slev1 pose", getPosition());
        board.addNum("slev2 pose", slave2.getEncoder().getPosition());
        board.addNum("maseter pose", master.getEncoder().getPosition());

        board.addNum("abs encoder", absEncoder.getAbsolutePosition());
        board.addNum("setPoint", getSetPoint());
        board.addNum("current", getCurrent());

        pidController.setP(pidGainSupplier.getKP());
        pidController.setI(pidGainSupplier.getKI());
        pidController.setD(pidGainSupplier.getKD());

        if (!DriverStation.isEnabled()){
            master.setIdleMode(Robot.getIsBreak());
            slave1.setIdleMode(Robot.getIsBreak());
            slave2.setIdleMode(Robot.getIsBreak());
        }
    }
}
