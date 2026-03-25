package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.wristConstants;

public class WristSubystem extends SubsystemBase { 
  SparkMax left_WristMotor = new SparkMax(wristConstants.leftMotorWrist_ID, MotorType.kBrushless);
  SparkMax right_WristMotor = new SparkMax(wristConstants.rightMotorWrist_ID, MotorType.kBrushless);
  SparkMaxConfig leftmotorConfig = new SparkMaxConfig();
  SparkMaxConfig rightmotorConfig = new SparkMaxConfig();
  public PIDController wristPID;

  public Command goToPostionVoltage(double position) {
  Command ejecutable =
    Commands.runOnce(
      () -> {
        wristPID.reset();
       wristPID.setSetpoint(position);
      },
      this
    );

    return ejecutable;
 }

  DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(wristConstants.encoder_ID);
  
  public WristSubystem() {
    leftmotorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);

    rightmotorConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);

    left_WristMotor.configure(leftmotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    right_WristMotor.configure(rightmotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    wristPID = new PIDController(Constants.wristConstants.kP, Constants.wristConstants.kI, Constants.wristConstants.kD);

    

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(" Wrist Absolute Encoder Angle", absoluteEncoder.get());

    left_WristMotor.set(wristPID.calculate(absoluteEncoder.get(), wristPID.getSetpoint()));
    right_WristMotor.set(0.775 * wristPID.calculate(absoluteEncoder.get(), wristPID.getSetpoint()));
  }

  public double getEnoderPosition(){
    return absoluteEncoder.get();
  }

  public void wristRun(double power) {
    left_WristMotor.set(power);
    right_WristMotor.set(power);
  }

}
