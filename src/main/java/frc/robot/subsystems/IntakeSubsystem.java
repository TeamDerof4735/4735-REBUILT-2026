package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeconstants;

public class IntakeSubsystem extends SubsystemBase {

  SparkMax intakeMotor = new SparkMax(intakeconstants.intakeMotor_ID, MotorType.kBrushless);
  SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
  
  public IntakeSubsystem() {
    intakeMotorConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(55);

    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {}

  public void intakeMove(double power) {
    intakeMotor.set(power);
  }

  public void intakeStop() {
    intakeMotor.set(0);
  }
}
