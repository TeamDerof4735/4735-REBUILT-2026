package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.indexerConstant;
import frc.robot.Constants.intakeconstants;

public class Intake extends SubsystemBase {

  SparkMax intakeMotor = new SparkMax(intakeconstants.intakeMotor_ID, MotorType.kBrushless);
  SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
  
  public Intake() {
    intakeMotorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);

    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {}

  public void intakeRun(double power) {
    intakeMotor.set(power);
  }

  public void intakeStop() {
    intakeMotor.set(0);
  }
}
