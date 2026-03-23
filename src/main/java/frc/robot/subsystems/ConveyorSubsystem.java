package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.conveyorConstant;

public class Conveyor extends SubsystemBase {

  SparkMax conveyorMotor = new SparkMax(conveyorConstant.conveyorMotor_ID, MotorType.kBrushless);
  SparkMaxConfig conveyorMotorConfig = new SparkMaxConfig();
  
  public Conveyor() {
    conveyorMotorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(45);

    conveyorMotor.configure(conveyorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {}

  public void conveyorRun() {
    conveyorMotor.set(0.7);
  }

  public void conveyorStop() {
    conveyorMotor.set(0);
  }
}
