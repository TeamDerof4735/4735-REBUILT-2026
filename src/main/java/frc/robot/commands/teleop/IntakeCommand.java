package frc.robot.commands.teleop;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  double power;

  public IntakeCommand(IntakeSubsystem intakeSubsystem, double power) {
    this.intakeSubsystem = intakeSubsystem;
    this.power = power;
    addRequirements(intakeSubsystem);
  }
  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    intakeSubsystem.intakeMove(power);
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