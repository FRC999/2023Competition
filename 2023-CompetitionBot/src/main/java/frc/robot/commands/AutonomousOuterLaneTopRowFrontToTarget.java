// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousOuterLaneTopRowFrontToTarget extends SequentialCommandGroup {
  /** Creates a new AutonomousOuterLaneTopRowFrontToTarget. 
   * At the start Front faces the target.
   * Place element on the top row.
   * Then drive forward to leave the home zone.
  */
  public AutonomousOuterLaneTopRowFrontToTarget() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("AutonomousOuterLaneTopRowFrontToTarget"),
      new PrintCommand("Placing Game element..."),
      new GPMAutoFarConeStraightFront(),
      new WaitCommand(0.2),
      new PrintCommand("Moving outside the home zone..."),
      new RunTrajectorySequenceRobotAtStartPoint("TopRowToTopGPForward",true) // go backwards towards the field center
    );
  }
}
