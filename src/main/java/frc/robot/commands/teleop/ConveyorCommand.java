package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorCommand extends Command {
  ConveyorSubsystem conveyor;

  public ConveyorCommand(ConveyorSubsystem conveyor) {
    this.conveyor = conveyor;
    addRequirements(conveyor);
  }
  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    conveyor.conveyorMove(0.3);
  }

  
  @Override
  public void end(boolean interrupted) {
    conveyor.conveyorMove(0);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
