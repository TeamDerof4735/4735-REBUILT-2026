package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.shooterConstant;

public class ShooterSubsystem extends SubsystemBase {

  SparkMax shooterMotor = new SparkMax(shooterConstant.shooterMotor_ID, MotorType.kBrushless);
  SparkMax indexMotor = new SparkMax(shooterConstant.indexMotor_ID, MotorType.kBrushless);
  SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();
  SparkMaxConfig indexMotorConfig = new SparkMaxConfig();
  SparkClosedLoopController shooterCloose = shooterMotor.getClosedLoopController();

  ShooterInterpolation shooterInterpolation;

  private RelativeEncoder shooterEncoder;

  
  public ShooterSubsystem() {
    shooterEncoder = shooterMotor.getEncoder();

    shooterMotorConfig
      .inverted(false)
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(90)
      .encoder.velocityConversionFactor(1);

    shooterMotorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(
      Constants.shooterConstant.left_kp,
      Constants.shooterConstant.left_ki,
      Constants.shooterConstant.left_kd)
      .outputRange(-1.0, 1.0);

    shooterMotorConfig.closedLoop.feedForward
      .kV(Constants.shooterConstant.left_F);

    shooterMotorConfig.encoder
      .uvwMeasurementPeriod(8);

    indexMotorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(45);

    shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    indexMotor.configure(indexMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter RPM", getShooterRPM());

    // Obtener pose del AprilTag en espacio de cámara
    double[] pose = LimelightHelpers.getTargetPose_CameraSpace("limelight-derof");

    if (pose != null && pose.length >= 3 && LimelightHelpers.getTV("limelight-derof")) {
      double x = pose[0]; // izquierda/derecha (metros)
      double y = pose[1]; // arriba/abajo (metros)
      double z = pose[2]; // frente (metros)

      // Distancia directa 3D
      double distanceMeters = z;

      SmartDashboard.putNumber("AprilTag X", x);
      SmartDashboard.putNumber("AprilTag Y", y);
      SmartDashboard.putNumber("AprilTag Z", z);
      SmartDashboard.putNumber("Distancia AprilTag (M)", distanceMeters);

    } else {
      SmartDashboard.putNumber("Distancia AprilTag (M)", 0);
    }
  }

  public double getShooterRPM () {
    return shooterEncoder.getVelocity();
  }

  public void shooterSpeed(double speedMotor) {
    shooterCloose.setSetpoint(speedMotor, ControlType.kVelocity);
  }

  public void indexMove(double power) {
    indexMotor.set(power);
  }

  
  // Interpolation Not Good
  public double calculateShoot(double distancia) {


    double dMin = 1.4;   // distancia mínima esperada (metros)
    double dMax = 5.3;   // distancia máxima esperada

    double rpmMin = 2000;   // potencia mínima
    double rpmMax = 5200;   // potencia máxima

    // Regla de 3 (interpolación lineal)
    double potencia = rpmMin + 
        ((distancia  - dMin) / (dMax - dMin)) * (rpmMax - rpmMin);

    return potencia;
  }

  
} 
