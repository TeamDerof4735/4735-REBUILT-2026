package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTests extends Command{
  ShooterSubsystem shooter;
  ConveyorSubsystem conveyor;
  Timer Timer = new Timer();

  public ShooterTests(ShooterSubsystem shooter, ConveyorSubsystem conveyor) {
    this.shooter = shooter;
    this.conveyor = conveyor;

    addRequirements(shooter);
    addRequirements(conveyor);
  }

  
  @Override
  public void initialize() {
    Timer.reset();
  }

  
  @Override
  public void execute() {
    shooter.shooterSpeed(1995);
    
    if(shooter.getShooterRPM() >= 1994){
      Timer.start();
      if (Timer.get() >= 0.4) {
        shooter.shooterSpeed(1995);
        shooter.indexMove(-0.7);
        conveyor.conveyorMove(0.7);
      } else {
        shooter.shooterSpeed(1995);
        shooter.indexMove(0);
        conveyor.conveyorMove(0);
      }
    } else {
      shooter.shooterSpeed(1995);
      shooter.indexMove(0);
      conveyor.conveyorMove(0);
      Timer.reset();
    }
  }

  
  @Override
  public void end(boolean interrupted) {
    shooter.shooterSpeed(0);
    shooter.indexMove(0);
    conveyor.conveyorMove(0);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
