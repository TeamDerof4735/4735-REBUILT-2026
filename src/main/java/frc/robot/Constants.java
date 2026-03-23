package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants
{

  

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }



public static final String limelightName = "limelight";  // Cambia si tu cámara tiene otro nombre en la red

// PID Gains (ajústalos según pruebas en el robot)
public static final double X_REEF_ALIGNMENT_P = 0.75;
public static final double Y_REEF_ALIGNMENT_P = 0.75;
public static final double ROT_REEF_ALIGNMENT_P = 0.5;

// Setpoints: valores objetivo que el robot debe alcanzar
public static final double X_SETPOINT_REEF_ALIGNMENT = 0.1; // Por ejemplo: distancia en metros al tag
public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.0; // Centrado horizontal
public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0.0; // Rotación frente al tag

// Tolerancias: márgenes de error aceptables
public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.01; // ±5 cm
public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;
public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1.0; // ±2 grados

// Tiempo mínimo que el robot debe mantenerse alineado
public static final double POSE_VALIDATION_TIME = 0.3; // segundos



  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class wristConstants
   {
    public static int leftMotorWrist_ID = 14; 
    public static int rightMotorWrist_ID = 15;
    public static int encoder_ID = 0;

    public static double guardado = 0.58;
    public static double afuera = 0.295;


    public static double kP = 3.15;
    public static double kI = 0;
    public static double kD = 0.35;

    
  }

  public static final class shooterConstant
   {
    public static int shooterMotor_ID = 10; 
    public static int indexMotor_ID = 11;

    public static double left_kp = 0.0095;       //0.000095
    public static double left_ki = 0;              //0
    public static double left_kd = 0.0000001;           //1.45
    public static double left_F = 0.00023;         //0.00021
    
  }

  public static final class conveyorConstant
  {
    public static int conveyorMotor_ID = 13; 
    
  }

  public static final class intakeconstants
   {
    public static int intakeMotor_ID = 12; 
  
    
  }


  public static final class VisionConstants {

    public static double xyStdDevCoefficient = 0.2;
    public static double thetaStdDevCoefficient = 0.4;

   
  }

  
  
  


  public static final double fieldBorderMargin = 0.01;
  public static final double fieldLength = 17.29;  //Meters
  public static final double fieldWidth = 7.78;
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
