// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousMidlaneBackToTargetPlace3rdRowBalanceFlontClimb18 extends SequentialCommandGroup {
  /** Creates a new AutonomousMidlaneBackToTargetPlace3rdRowBalanceFlontClimb18.
   * Start with Back to target.
   * Place CUBE in 3rd row.
   * Balance front-forward (without leaving home zone)
   */
  public AutonomousMidlaneBackToTargetPlace3rdRowBalanceFlontClimb18() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("AutonomousMidlaneBackToTargetPlace3rdRowBalance"),
      new PrintCommand("Placing Game element..."),
      new GPMAutoHighCubeStraightBackUltimate(), // that will take us off the pin as well
      new SelfBalanceWhenFacingTheCharger(0.3, 0, true, true)
 
    );
  }
}
