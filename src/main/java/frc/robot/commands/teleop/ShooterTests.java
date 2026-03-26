package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTests extends Command{
  ShooterSubsystem shooter;
  ConveyorSubsystem conveyor;
  Timer Timer = new Timer();
  double rpm;

  public ShooterTests(ShooterSubsystem shooter, ConveyorSubsystem conveyor, double rpm) {
    this.shooter = shooter;
    this.conveyor = conveyor;
    this.rpm = rpm;

    addRequirements(shooter);
    addRequirements(conveyor);
  }

  
  @Override
  public void initialize() {
    Timer.reset();
  }

  
  @Override
  public void execute() {
    shooter.setShooterRPM(rpm);
    
    if(shooter.getCurrentRPM() >= (rpm-40)){
      Timer.start();
      if (Timer.get() >= 0.4) {
        shooter.setShooterRPM(rpm);
        shooter.indexMove(-0.7);
        conveyor.conveyorMove(0.95);
      } else {
        shooter.setShooterRPM(rpm);
        shooter.indexMove(0);
        conveyor.conveyorMove(0);
      }
    } else {
      shooter.setShooterRPM(rpm);
      shooter.indexMove(0);
      conveyor.conveyorMove(0);
      Timer.reset();
    }
  }

  
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterRPM(0);
    shooter.indexMove(0);
    conveyor.conveyorMove(0);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
