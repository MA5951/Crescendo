package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.MAShuffleboard.pidControllerGainSupplier;
import com.ma5951.utils.subsystem.DefaultInternallyControlledSubsystem;

import frc.robot.PortMap;

public class Elevator extends SubsystemBase implements DefaultInternallyControlledSubsystem{

    private static Elevator elevator;

    private final AnalogEncoder absEncoder;

    private final CANSparkMax master;
    private final CANSparkMax slave1;
    private final CANSparkMax slave2;
    private final RelativeEncoder encoder;

    private final SparkPIDController pidController;
    private double setPoint = 0;

    private final pidControllerGainSupplier pidGainSupplier;

    private final MAShuffleboard board;


    private Elevator() {
      master = new CANSparkMax(PortMap.Elevator.masterID, MotorType.kBrushless);
      slave1 = new CANSparkMax(PortMap.Elevator.slave1ID, MotorType.kBrushless);
      slave2 = new CANSparkMax(PortMap.Elevator.slave2ID, MotorType.kBrushless);

      absEncoder = new AnalogEncoder(PortMap.Elevator.absEncoderID);

      master.setIdleMode(IdleMode.kBrake);
      slave1.setIdleMode(IdleMode.kBrake);
      slave2.setIdleMode(IdleMode.kBrake);
      slave1.follow(master, true);
      slave2.follow(master, true);

      encoder = master.getEncoder();

      encoder.setPositionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR);

      resetPose(0);

      pidController = master.getPIDController();
      pidController.setFeedbackDevice(encoder);
      pidController.setP(ElevatorConstants.KP);
      pidController.setI(ElevatorConstants.KI);
      pidController.setD(ElevatorConstants.KD);


      slave1.follow(master, true);
      slave2.follow(master, true);

      board = new MAShuffleboard("Elevator");
      pidGainSupplier = board.getPidControllerGainSupplier(
        "position",
        ElevatorConstants.KP,
        ElevatorConstants.KI,
        ElevatorConstants.KD);

        board.addNum("setPoint", getSetPoint());

    }

    public double getCurrent() {
        return master.getOutputCurrent();
    }

    public double getPoseForShoot() {
        return 0; // TODO graph
    }

    @Override
    public void calculate(double setPoint) {
        pidController.setReference(setPoint, ControlType.kPosition);
    }

    @Override
    public boolean atPoint() {
        return Math.abs(getPosition() - getSetPoint()) <= ElevatorConstants.TOLERANCE;
    }

    @Override
    public void setSetPoint(double setPoint) {
        board.addNum("setPoint", setPoint);
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
        return setPoint >= ElevatorConstants.MIN_POSE &&
         setPoint <= ElevatorConstants.MAX_POSE;
    }

    public static Elevator getInstance() {
        if (elevator == null) {
          elevator = new Elevator();
        }
        return elevator;
    }

    @Override
    public void periodic() {

        board.addNum("abs encoder", absEncoder.getAbsolutePosition());
        board.addNum("current", getCurrent());

        board.addNum("pose", getPosition());

        board.addBoolean("at point", atPoint());

        pidController.setP(pidGainSupplier.getKP());
        pidController.setI(pidGainSupplier.getKI());
        pidController.setD(pidGainSupplier.getKD());

        setSetPoint(board.getNum("setPoint"));

        board.addNum("v", encoder.getVelocity());
    }
}
