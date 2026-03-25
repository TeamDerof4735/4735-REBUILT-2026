package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivebase.AutoAim;
import frc.robot.commands.teleop.IntakeCommand;
import frc.robot.commands.teleop.WristFeedingCommand;
import frc.robot.commands.teleop.WristTests;
import frc.robot.commands.teleop.ShooterCommand;
import frc.robot.commands.teleop.ShooterTests;
//import frc.robot.commands.teleop.ShooterTests;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.WristSubystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterInterpolation;
import frc.robot.subsystems.ShooterSubsystem;

import java.io.File;
import swervelib.SwerveInputStream;


public class RobotContainer
{
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                      "neo"));

  // Subsystems
  private final WristSubystem wristSubystem = new WristSubystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Virtual Subsystems
  private final ShooterInterpolation shooterInterpolation = new ShooterInterpolation();
  private final Vision vision = new Vision(drivebase, shooterInterpolation);

  // Commands
  private WristFeedingCommand intakeFeedingCommand = new WristFeedingCommand(wristSubystem);
  private SendableChooser<Command> auto = new SendableChooser<>();

  //Controls :)                                                                                    
  private static final CommandXboxController driverController = new CommandXboxController(0);
  private static final CommandXboxController subsystemController = new CommandXboxController(1);

  
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * -1,
                                                                () -> driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverController.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightX,
  driverController::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverController.getLeftY(),
                                                                        () -> -driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverController.getRightX())
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);


  public RobotContainer()
  {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Auto Selection
    SmartDashboard.putData("Select Auto", auto);
    auto.addOption("nada", null);
  }


  private void configureBindings()
  {
    // ----------------- Control Chassis -----------------
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
      driverController.rightBumper().onTrue(drivebase.driveToPose(new Pose2d(11, 5, Rotation2d.fromDegrees(drivebase.desiredHeadingDeg()))));
    } 
      else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(10.75, 4.2, new Rotation2d()))));
      driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    }

    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverController.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverController.back().whileTrue(drivebase.centerModulesCommand());
      driverController.leftBumper().onTrue(Commands.none());
    } 
      else
    {
      driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
                              driverController.start().whileTrue(Commands.none());
                              driverController.back().whileTrue(Commands.none());
                              driverController.b().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    }

    // ----------------- Control Subsysem -----------------     
    driverController.rightBumper().whileTrue(new SequentialCommandGroup( //INTAKE ABAJO L1
      wristSubystem.goToPostionVoltage(Constants.wristConstants.afuera),
      new IntakeCommand(intakeSubsystem)    
    ));

    driverController.y().onTrue(new SequentialCommandGroup( //INTAKE ABAJO L1
      wristSubystem.goToPostionVoltage(Constants.wristConstants.guardado)   
    ));

    subsystemController.b().whileTrue(new WristTests(wristSubystem));

    driverController.x().onTrue(new AutoAim(drivebase, 
    vision, 
    ()->MathUtil.applyDeadband(-driverController.getLeftY(), 0.05), 
    ()->MathUtil.applyDeadband(-driverController.getLeftX(), 0.05)));
    
    //subsystemController.a().whileTrue( new ShooterTests(shooterSubsystem, conveyorSubsystem)); //Shooter Testing
    driverController.leftBumper().whileTrue( new ShooterCommand(shooterSubsystem, conveyorSubsystem, shooterInterpolation, intakeFeedingCommand, vision)); //Shooter Command
    
  }


  public Command getAutonomousCommand()
  {
    return auto.getSelected();
  }
}
