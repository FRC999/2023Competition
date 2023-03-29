// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.NavigationConstants;
import frc.robot.Constants.GamepieceManipulator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GPMAutoMidCubeStraightBackV2Parallel extends SequentialCommandGroup {
  /** Creates a new GPMAutoMidCubeStraightBackV2.
   * Start with back to the target.
   * Place CUBE in the second position (mid-row) using Arm/Elevator Parallel routine
   * Retract to semi-fetal position.
   */
  public GPMAutoMidCubeStraightBackV2Parallel() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new PrintCommand("Auto-cube 2nd row"),
        // new TurretTurnToFront(),
        // new PrintCommand("Turret Done"),
        new GPMAutoParallelMidElevatorArmCube(),
        new WaitCommand(0.3),
        new FlipperDown(),
        new WaitCommand(0.5),
        new ClawOpen(),
        new WaitCommand(0.3),
        new FlipperUp(),
        new GPMSemiFetalPosition());
  }
}
