package frc.robot.commands.Intake;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends Command {
  private Intake intake;
  private MotorCommand motorCommand;
  
  public IntakeCommand(double power) {
    intake = Intake.getInstance();
    motorCommand = new MotorCommand(intake, power, 0);
  }

  @Override
  public void initialize() {
    motorCommand.initialize();
  }

  @Override
  public void execute() {
    motorCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    motorCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return intake.isGamePieceInIntake(); //canMove instand? 
  }
}
