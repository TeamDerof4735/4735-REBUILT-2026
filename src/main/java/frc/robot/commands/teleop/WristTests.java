package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubystem;

public class WristTests extends Command {
  WristSubystem wristSubystem;
  
  public WristTests(WristSubystem wristSubystem) {
  this.wristSubystem = wristSubystem;

  addRequirements(wristSubystem);
}

  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    wristSubystem.wristRun(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    wristSubystem.wristRun(0);
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
