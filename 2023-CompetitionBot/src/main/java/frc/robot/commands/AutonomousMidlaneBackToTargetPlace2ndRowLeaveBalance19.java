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
public class AutonomousMidlaneBackToTargetPlace2ndRowLeaveBalance19 extends SequentialCommandGroup {
  /** Creates a new AutonomousMidlaneBackToTargetPlace2ndRowLeaveBalance.
   * Start with Back to target.
   * Place Element in 2nd row.
   * Leave home zone
   * Balance by returning
   */
  public AutonomousMidlaneBackToTargetPlace2ndRowLeaveBalance19() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("AutonomousMidlaneBackToTargetPlace2ndRowLeaveBalance"),
      new PrintCommand("Placing CUBE..."),
      new GPMAutoMidCubeStraightBackV2Parallel(), // that will take us off the pin as well
      new PrintCommand("Starting Trajectory.."),
      new RunTrajectorySequenceRobotAtStartPoint("MiddleRowOverCharger", 1.6, 1.3, false),
      new PrintCommand("Trajectory Done"),
      new WaitCommand(1.0), // wait until the charger quiets down
      new SelfBalanceWhenFacingTheCharger(0.3, 0, false, false)
    );
  }
}
