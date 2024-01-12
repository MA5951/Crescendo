package frc.robot.commands.Intake;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeCommand extends Command {
  private Intake intake;
  private MotorCommand intakeMotorCommand;
  
  public IntakeCommand() {
    intake = Intake.getInstance();
    intakeMotorCommand = new MotorCommand(intake, IntakeConstants.intakePower, 0);
  }

  @Override
  public void initialize() {
    intakeMotorCommand.initialize();
  }

  @Override
  public void execute() {
    intakeMotorCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    intakeMotorCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return intake.getHighSensor();
  }
}
