package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.IntakeandIndex;
//import frc.robot.subsystems.shooter;
//import frc.robot.subsystems.shooter;
import frc.robot.subsystems.Shooter;

public class RunShoot extends Command{

    Shooter shooter;
    IntakeandIndex intakeandIndex;
    Timer Timer = new Timer();

    public RunShoot(Shooter shooter, IntakeandIndex intakeandIndex) {
    this.shooter = shooter;
    this.intakeandIndex = intakeandIndex;
    addRequirements(shooter);
    addRequirements(intakeandIndex);
  }

  
  @Override
  public void initialize() {
    Timer.reset();
  }

  
  @Override
  public void execute() {
    shooter.shootVel(1995);
    
    if(shooter.left_rpm() >= 1994){
      Timer.start();
      if (Timer.get() >= 0.4) {
        shooter.index(-0.7);
      } else {
        shooter.conveyorPOT(0);
      }
    } else {
      shooter.conveyorPOT(0);
      Timer.reset();
    }
  }

  
  @Override
  public void end(boolean interrupted) {
    shooter.shooterRun(0);
    shooter.conveyorPOT(0);
    intakeandIndex.runIND(0);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
