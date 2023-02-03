// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutonomousCommandPlaceholder;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.OperateTurret;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PigeonIMUSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();

  public static final PigeonIMUSubsystem pigeonIMUSubsystem = new PigeonIMUSubsystem();

  public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  public static final TurretSubsystem turretSubsystem = new TurretSubsystem(); 

  public static Joystick driveStick;
  public static Joystick turnStick;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure driver interface - joysticks
    configureDriverInterface();

    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    driveSubsystem.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        // new RunCommand(() ->
        // driveSubsystem.arcadeDrive(joystick.getLeftY(),
        // joystick.getRightX()), driveSubsystem));
        new DriveManuallyCommand());
        
  }

    /**
   * Use this method to define your controllers depending on the
   * {@link DriveInterface}
   */
  private void configureDriverInterface() {
    turnStick = new Joystick(OIConstants.turnControllerPort);
    driveStick = new Joystick(OIConstants.driverControllerPort);

    System.out.println("Driver interface configured");
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

    new JoystickButton(driveStick, 10)
          .whileTrue(new OperateTurret())
          .whileFalse(new InstantCommand(RobotContainer.turretSubsystem::stopTurret, RobotContainer.turretSubsystem));
    
    new JoystickButton(driveStick, 7)
          .whileTrue(new OperateTurret())
          .whileFalse(new InstantCommand(RobotContainer.elevatorSubsystem::stopElevator, RobotContainer.elevatorSubsystem));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new AutonomousCommandPlaceholder();
  }
}
