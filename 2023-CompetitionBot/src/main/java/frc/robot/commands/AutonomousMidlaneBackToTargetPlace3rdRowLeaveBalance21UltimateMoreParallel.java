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
public class AutonomousMidlaneBackToTargetPlace3rdRowLeaveBalance21UltimateMoreParallel extends SequentialCommandGroup {
  /** Creates a new AutonomousMidlaneBackToTargetPlace2ndRowLeaveBalance21UltimateMoreParallel. */
  public AutonomousMidlaneBackToTargetPlace3rdRowLeaveBalance21UltimateMoreParallel() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("AutonomousMidlaneBackToTargetPlace3rdRowLeaveBalance21UltimateMoreParallel"),
      new PrintCommand("Placing CUBE..."),
      new GPMAutoHighCubeStraightBackUltimateV3WithoutRetraction(), // that will take us off the pin as well
      new PrintCommand("Starting Retraction and Trajectory.."),
      new AutoParallelRetractionAndTrajectoryOverChargerFrontForward(),
      new PrintCommand("Trajectory Done"),
      new WaitCommand(1.0), // wait until the charger quiets down
      new SelfBalanceWhenFacingTheCharger(0.3, 0, false, false)
 
    );
  }
}
