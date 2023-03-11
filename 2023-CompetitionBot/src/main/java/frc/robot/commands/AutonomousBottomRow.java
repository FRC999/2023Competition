// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousBottomRow extends SequentialCommandGroup {
  /** Creates a new AutonomousBottomRow. */
  public AutonomousBottomRow() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonomousGamepieceSecondRowWhenFacingBack(),
      new TurretToAngle(-3),
      // add trajectory
      new PrintCommand("Starting Trajectory.."),
      new RunTrajectorySequenceRobotAtStartPoint("BottomRowToBottomGFForward"),
      new PrintCommand("Trajectory Done"),
      new ArmToLength(0.7),
      new WaitCommand(0.3),
      new InstantCommand(RobotContainer.clawSubsystem::flipperDown),
      new WaitCommand(0.2),
      new InstantCommand(RobotContainer.clawSubsystem::closeClaw),
      new WaitCommand(0.2),
      new InstantCommand(RobotContainer.clawSubsystem::flipperUp),
      new WaitCommand(0.2),
      new RunTrajectorySequenceRobotAtStartPoint("BottomRowToBottomGFForward",true) // go backwards to the original location
    );
  }
}
