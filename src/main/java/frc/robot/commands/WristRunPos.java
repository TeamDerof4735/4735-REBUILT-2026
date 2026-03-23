// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.wristConstants;
//import frc.robot.subsystems.WristSubystem;
import frc.robot.subsystems.WristSubystem;

public class WristRunPos extends Command {
/* 

  WristSubystem wristSubystem;
  private PIDController wristPID;
  private double setPoint;
  
  public WristRunPos(WristSubystem wristSubystem, double setPoint, PIDController wristPID) {
    this.wristSubystem = wristSubystem;
    this.setPoint = setPoint;
    this.wristPID = wristPID;
    addRequirements(wristSubystem);

  }

  @Override
  public void initialize() {

    wristPID.reset();
    wristPID.setSetpoint(setPoint);
    
  }

  
  @Override
  public void execute() {
    wristSubystem.wristRun(wristPID.calculate(wristSubystem.encoderPos(), setPoint));
  }

  
  @Override
  public void end(boolean interrupted) {
    wristSubystem.wristRun(0);
  }

  
  @Override
  public boolean isFinished() {
    return wristPID.atSetpoint();
  }
    */
}
