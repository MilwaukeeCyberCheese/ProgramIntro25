// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_drive  = new SwerveSubsystem(Constants.SwerveConstants.kSwerveJsonDirectory);
                                                                            
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_drive.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(m_driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                           .headingWhile(true);

  public final AutoChooser m_autoChooser = new AutoChooser();
  private final AutoFactory m_autoFactory =
      new AutoFactory(
          m_drive::getPose, m_drive::resetOdometry, m_drive::followTrajectory, true, m_drive);

  private AutoRoutine getTestAuto1() { 
    AutoRoutine r = m_autoFactory.newRoutine("Test Auto 1");
    AutoTrajectory mainTraj = r.trajectory("Test Auto 1");
    r.active().onTrue(Commands.sequence(mainTraj.resetOdometry(), mainTraj.cmd()));
    return r;
  }

  private AutoRoutine getTestAutoExample() {
    AutoRoutine r = m_autoFactory.newRoutine("Test Auto Example");
    AutoTrajectory mainTraj = r.trajectory("Test Auto Example");
    r.active().onTrue(Commands.sequence(mainTraj.resetOdometry(), mainTraj.cmd()));
    return r;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    m_autoChooser.addRoutine("Test Auto 1", this::getTestAuto1);
    m_autoChooser.addRoutine("Test Auto Example", this::getTestAutoExample);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle      = m_drive.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = m_drive.driveFieldOriented(driveAngularVelocity);
    Command driveSetpointGen = m_drive.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);

    m_drive.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    m_driverController.a().onTrue((Commands.runOnce(m_drive::zeroGyro)));
    m_driverController.x().onTrue(Commands.runOnce(m_drive::addFakeVisionReading));
    m_driverController.start().whileTrue(Commands.none());
    m_driverController.back().whileTrue(Commands.none());
    m_driverController.leftBumper().whileTrue(Commands.runOnce(m_drive::lock, m_drive).repeatedly());
    m_driverController.rightBumper().onTrue(Commands.none());

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
