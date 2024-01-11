package frc.robot.subsystems.conveyor;

import com.ma5951.utils.subsystem.MotorSubsystem;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class Conveyor extends SubsystemBase implements MotorSubsystem {

    private static Conveyor conveyor;

    private CANSparkMax motor;

    private Conveyor() {
        motor = new CANSparkMax(PortMap.Conveyor.motorID, MotorType.kBrushless);
    }

    @Override
    public boolean canMove() {
        return true;
    }

    @Override
    public void setVoltage(double voltage) {
        motor.set(voltage / 12);
    }

    public static Conveyor getInstance() {
        if (conveyor == null) {
            conveyor = new Conveyor();
        }
        return conveyor;
    }    

    @Override
    public void periodic() {
    }
}