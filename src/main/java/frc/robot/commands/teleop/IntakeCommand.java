package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  IntakeSubsystem intakeSubsystem;

  public IntakeCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }
  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    intakeSubsystem.intakeMove(0.85);
  }

  
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.intakeMove(0);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}