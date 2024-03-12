package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SwerveModuleTalonFX extends SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.DRIVE_KS,
            SwerveConstants.DRIVE_KV);

    private final CANcoder absoluteEcoder;

    private final boolean isAbsoluteEncoderReversed;
    private final double offsetEncoder;

    private boolean isDriveMotorReversed;
    private boolean isTurningMotorReversed;

    private PositionVoltage angleSetter = new PositionVoltage(0);
    private VelocityVoltage velocitySetter = new VelocityVoltage(0);

    private StatusSignal<Double> drivePosition;
    private StatusSignal<Double> driveVelocity;
    private StatusSignal<Double> steerPosition;
    private StatusSignal<Double> absAngle;

    private double angleOffset;
    private double dirvePoseOffset;

    public SwerveModuleTalonFX(String tabName, int driveID,
            int turningID, int absoluteEncoderID, boolean isDriveMotorReversed,
            boolean isTurningMotorReversed, boolean isAbsoluteEncoderReversed,
            double offsetEncoder, String canbus) {
        this.absoluteEcoder = new CANcoder(absoluteEncoderID, canbus);

        this.offsetEncoder = offsetEncoder;
        this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed;

        this.isDriveMotorReversed = isDriveMotorReversed;
        this.isTurningMotorReversed = isTurningMotorReversed;

        this.driveMotor = new TalonFX(driveID, canbus);
        this.turningMotor = new TalonFX(turningID, canbus);

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        steerPosition = turningMotor.getPosition();

        absAngle = absoluteEcoder.getAbsolutePosition();
        
        configTurningMotor();
        configDriveMotor();
        resetEncoders();
    }

    private void configTurningMotor() {
        TalonFXConfiguration turningConfiguration = new TalonFXConfiguration();

        turningConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        turningConfiguration.ClosedLoopGeneral.ContinuousWrap = false;

        turningConfiguration.MotorOutput.Inverted = 
            isTurningMotorReversed ? InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;

        turningConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        turningConfiguration.Slot0.kP = SwerveConstants.TURNING_PID_KP;
        turningConfiguration.Slot0.kI = SwerveConstants.TURNING_PID_KI;
        turningConfiguration.Slot0.kD = SwerveConstants.TURNING_PID_KD;
        turningConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 
            SwerveConstants.TURNING_PEAK_CURRENT_LIMIT_TORQUE_CURRENT;
        turningConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = 
            -SwerveConstants.TURNING_PEAK_CURRENT_LIMIT_TORQUE_CURRENT;

        turningConfiguration.CurrentLimits.SupplyCurrentLimitEnable = 
            SwerveConstants.TURNING_ENABLE_CURRENT_LIMIT;
        turningConfiguration.CurrentLimits.SupplyCurrentLimit =
            SwerveConstants.TURNING_CONTINUOUS_CURRENT_LIMIT;
        turningConfiguration.CurrentLimits.SupplyCurrentThreshold =
            SwerveConstants.TURNING_PEAK_CURRENT_LIMIT;
        turningConfiguration.CurrentLimits.SupplyTimeThreshold = 
            SwerveConstants.TURNING_PEAK_CURRENT_DURATION;
        
        turningMotor.getConfigurator().apply(turningConfiguration);
    }

    private void configDriveMotor() {
        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();

        driveConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        driveConfiguration.ClosedLoopGeneral.ContinuousWrap = false;

        driveConfiguration.MotorOutput.Inverted = 
            isDriveMotorReversed ? InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;

        driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveConfiguration.Slot0.kP = SwerveConstants.DRIVE_PID_KP;
        driveConfiguration.Slot0.kI = SwerveConstants.DRIVE_PID_KI;
        driveConfiguration.Slot0.kD = SwerveConstants.DRIVE_PID_KD;

        driveConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 
            SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT_TORQUE_CURRENT;
        driveConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = 
            -SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT_TORQUE_CURRENT;

        driveConfiguration.CurrentLimits.SupplyCurrentLimitEnable = 
            SwerveConstants.DRIVE_ENBLE_CURRENT_LIMIT;
        driveConfiguration.CurrentLimits.SupplyCurrentLimit = 
            SwerveConstants.DRIVE_CONTINUOS_CURRENT_LIMIT;
        driveConfiguration.CurrentLimits.SupplyCurrentThreshold = 
            SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT;
        driveConfiguration.CurrentLimits.SupplyTimeThreshold = 
            SwerveConstants.DRIVE_PEAK_CURRENT_DURATION;

        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    public void resetEncoders() {
        angleOffset = ((getAbsoluteEncoderPosition() - offsetEncoder)
            / SwerveConstants.ANGLE_PER_PULSE) - 
            steerPosition.getValue();
        dirvePoseOffset = - drivePosition.getValue();
    }

    public double getCurrent() {
        return driveMotor.getStatorCurrent().getValue();
    }

    public double getAbsoluteEncoderPosition() {
        absAngle.refresh();
        return isAbsoluteEncoderReversed ? 360 - ((absAngle.getValue() + 0.5)) * 360
                : ((absAngle.getValue() + 0.5)) * 360;
    }

    public double getDrivePosition() {
        drivePosition.refresh();
        return drivePosition.getValue()
                * SwerveConstants.DISTANCE_PER_PULSE
                + dirvePoseOffset * SwerveConstants.DISTANCE_PER_PULSE;
    }

    public double getTurningPosition() {
        steerPosition.refresh();
        return (steerPosition.getValue()
                * SwerveConstants.ANGLE_PER_PULSE) + 
                angleOffset * SwerveConstants.ANGLE_PER_PULSE;
    }

    public double getDriveVelocity() {
        driveVelocity.refresh();
        return (driveVelocity.getValue()
                * SwerveConstants.DISTANCE_PER_PULSE) /
                SwerveConstants.VELOCITY_TIME_UNIT_IN_SECONDS;
    }

    public void setAccelerationLimit(double limit) {
        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
        ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs();

        closedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = limit;

        driveMotor.getConfigurator().apply(driveConfiguration);
    }

    public void turningMotorSetPower(double power) {
        turningMotor.set(power);
    }

    public void driveMotorSetPower(double power) {
        driveMotor.set(power);
    }

    public void setInvertedTurning(Boolean turningMode) {
        turningMotor.setInverted(turningMode);
    }

    public void turningUsingPID(double setPoint) {
        turningMotor.setControl(
                angleSetter.withPosition((setPoint
                        / SwerveConstants.ANGLE_PER_PULSE) - angleOffset)
                        .withSlot(0));
    }

    public void driveUsingPID(double setPoint) {
        driveMotor.setControl(
                velocitySetter.withFeedForward(
                        feedforward.calculate(
                                setPoint))
                        .withSlot(0).withVelocity((setPoint
                                / SwerveConstants.DISTANCE_PER_PULSE) /
                                SwerveConstants.VELOCITY_TIME_UNIT_IN_SECONDS));
    }
}