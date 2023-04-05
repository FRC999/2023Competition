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
public class AutonomousMidlaneBackToTargetPlace2ndRowLeaveBalance19MoreParallel extends SequentialCommandGroup {
  /** Creates a new AutonomousMidlaneBackToTargetPlace2ndRowLeaveBalance19MoreParallel. */
  public AutonomousMidlaneBackToTargetPlace2ndRowLeaveBalance19MoreParallel() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("AutonomousMidlaneBackToTargetPlace2ndRowLeaveBalance19MoreParallel"),
      new PrintCommand("Placing CUBE..."),
      new GPMAutoMidCubeStraightBackV3ParallelWithoutRetraction(), // that will take us off the pin as well
      new PrintCommand("Starting Retraction and Trajectory.."),
      new AutoParallelRetractionAndTrajectoryOverChargerFrontForward(),
      new PrintCommand("Trajectory Done"),
      new WaitCommand(1.0), // wait until the charger quiets down
      new SelfBalanceWhenFacingTheCharger(0.3, 0, false, false)
 
    );
  }
}
