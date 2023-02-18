// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutonomousCommandPlaceholder;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.OperateTurret;
import frc.robot.commands.RunTrajectorySequenceRobotAtStartPoint;
import frc.robot.commands.SYSIDMoveForward;
import frc.robot.commands.SelfBalanceWhenFacingTheCharger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BigFootSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.NetworkTablesSubsystem;
import frc.robot.subsystems.PigeonIMUSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  // ***** Initialize Subsystems *******

  public static final PigeonIMUSubsystem pigeonIMUSubsystem = new PigeonIMUSubsystem();

  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();

  // GamePiece Manipulator subsystems
  //public static final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  //public static final TurretSubsystem turretSubsystem = new TurretSubsystem();
  //public static final ArmSubsystem armSubsystem = new ArmSubsystem();
  // The next two are pneumatically operated, so the PneumaticsSubsystem, which starts the compressor, should be initialized first

  public static final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
  
  //public static final ClawSubsystem clawSubsystem = new ClawSubsystem();

  // Foot that stops us when balanced
  //public static final BigFootSubsystem bigFootSubsystem = new BigFootSubsystem();
  // ***** End of Subsystem initialization *******

  // ***** Dummy subsystems so the code will compile, use for testing
  //  !!!!!!! Make sure to comment it out for thre real competition
  //public static final PigeonIMUSubsystem pigeonIMUSubsystem = null;
  //public static final DriveSubsystem driveSubsystem = null;
  public static final ElevatorSubsystem elevatorSubsystem = null;
  public static final TurretSubsystem turretSubsystem = null;
  public static final ArmSubsystem armSubsystem = null;
  //public static final PneumaticsSubsystem pneumaticsSubsystem = null;
  public static final ClawSubsystem clawSubsystem = null;
  public static final BigFootSubsystem bigFootSubsystem = null;
  public static final NavigationSubsystem navigationSubsystem = null;
  public static final NetworkTablesSubsystem networkTablesSubsystem = new NetworkTablesSubsystem();

  public static final SmartDashboardSubsystem smartDashboardSubsystem = new SmartDashboardSubsystem(); 


  public static Joystick driveStick;
  public static Joystick turnStick;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure driver interface - binding joystick objects to port numbers
    configureDriverInterface();

    // Configure the trigger/button bindings for commands
    configureButtonBindings();

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
  private void configureButtonBindings() { 

    //TODO: all of the current commands and triggers are related to unit testing. Make sure to put real commands here
    /* 
    new JoystickButton(driveStick, 10)
          .whileTrue(new OperateTurret())
          .whileFalse(new InstantCommand(RobotContainer.turretSubsystem::stopTurret, RobotContainer.turretSubsystem));
    */
    /* 
    
    new JoystickButton(driveStick, 7)
          .whileTrue(new OperateTurret())
          .whileFalse(new InstantCommand(RobotContainer.elevatorSubsystem::stopElevator, RobotContainer.elevatorSubsystem));
    */
    // Solenoid test with the BigFoot subsystem
    
    /* 
    new JoystickButton(driveStick, 11)
          .whileTrue(new InstantCommand(RobotContainer.bigFootSubsystem::footDown,RobotContainer.bigFootSubsystem))
          .whileFalse(new InstantCommand(RobotContainer.bigFootSubsystem::footUp,RobotContainer.bigFootSubsystem));
    */
    // Simple Trajectory test
    
    new JoystickButton(driveStick, 12)
          .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("trajtest1"))
          .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem, RobotContainer.pigeonIMUSubsystem));
    
    // SYSID Test
    
    new JoystickButton(driveStick, 9)
          .whileTrue(new RunTrajectorySequenceRobotAtStartPoint("SYSIDMoveForwardTest"))
          .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem, RobotContainer.pigeonIMUSubsystem));
          

    // Self-Balance test
    
    new JoystickButton(driveStick, 10)
      .whileTrue(new SelfBalanceWhenFacingTheCharger(0.5, 0, true, true))
      .whileFalse(new InstantCommand(RobotContainer.driveSubsystem::stopRobot, RobotContainer.driveSubsystem, RobotContainer.pigeonIMUSubsystem));
    
    
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
