package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut; // <--- Importante para el Coast
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstant;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX master = new TalonFX(Constants.shooterConstant.shooterMotor_ID);
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    // Requests: Uno para girar con PID y otro para dejarlo suelto (Coast)
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut coastRequest = new NeutralOut(); 

    private final SparkMax indexMotor = new SparkMax(shooterConstant.indexMotor_ID, MotorType.kBrushless);
    private final SparkMaxConfig indexMotorConfig = new SparkMaxConfig();

    private double targetRPM = 0.0;

    public ShooterSubsystem() {
        // Configuración del PID para el Kraken
        config.Slot0.kP = Constants.shooterConstant.left_kp;
        config.Slot0.kI = Constants.shooterConstant.left_ki;
        config.Slot0.kD = Constants.shooterConstant.left_kd;
        config.Slot0.kV = Constants.shooterConstant.left_kv;

        // Límites de corriente para no quemar nada
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // ESTO ES LO QUE FALTABA: Configurar el modo Neutral y APLICARLO
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        master.getConfigurator().apply(config);

        // Configuración del Indexer (SparkMax)
        indexMotorConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(45);

        indexMotor.configure(indexMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        // Aquí ya NO ponemos configuración de NeutralMode para no saturar el CAN
        SmartDashboard.putNumber("Shooter RPM", getCurrentRPM());
        SmartDashboard.putNumber("Shooter Target", getTargetRPM());
    }

    public void setShooterRPM(double rpm) {
        targetRPM = rpm;
        
        if (rpm <= 0) {
            // Si la meta es 0, mandamos "Coast" para que deje de vibrar y gire libre
            master.setControl(coastRequest);
        } else {
            // Si la meta es > 0, activamos el control de velocidad
            double rps = rpm / 60.0;
            master.setControl(velocityRequest.withVelocity(rps));
        }
    }

    public double getCurrentRPM() {
        return master.getVelocity().getValueAsDouble() * 60.0;
    }

    public void indexMove(double power) {
        indexMotor.set(power);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean atSpeed(double toleranceRPM) {
        return Math.abs(getCurrentRPM() - targetRPM) <= toleranceRPM;
    }

    // Interpolación para el disparo
    public double calculateShoot(double distancia) {
        double dMin = 1.4;   
        double dMax = 5.3;   
        double rpmMin = 2000;
        double rpmMax = 5200;

        double potencia = rpmMin + 
            ((distancia - dMin) / (dMax - dMin)) * (rpmMax - rpmMin);

        return potencia;
    }
}