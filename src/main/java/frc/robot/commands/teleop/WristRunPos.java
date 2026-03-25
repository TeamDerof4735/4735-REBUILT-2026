package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubystem;

public class WristRunPos extends Command {
  WristSubystem wristSubystem;
  PIDController wristPID;
  double wristKP;
  double wristKI;
  double wristKD;
  double setPoint;
  
  public WristRunPos(WristSubystem wristSubystem, double setPoint, double wristKP, double wristKI, double wristKD) {
  this.wristSubystem = wristSubystem;
  this.setPoint = setPoint;

  wristPID = new PIDController(wristKP, wristKI, wristKD);

  addRequirements(wristSubystem);
}

  @Override
  public void initialize() {
    wristPID.reset();
    wristPID.setSetpoint(setPoint);
    wristPID.setTolerance(0.4);
  }
  
  @Override
  public void execute() {
    wristSubystem.wristRun(wristPID.calculate(wristSubystem.getEnoderPosition(), setPoint));
  }

  @Override
  public void end(boolean interrupted) {
    wristSubystem.wristRun(0);
  }
  
  @Override
  public boolean isFinished() {
    return wristPID.atSetpoint();
  }
}
