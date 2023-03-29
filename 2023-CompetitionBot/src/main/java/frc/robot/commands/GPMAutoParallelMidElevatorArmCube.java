// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.NavigationConstants;
import frc.robot.commands.ElevatorToPredefinedHeight;
import frc.robot.Constants.GamepieceManipulator.Elevator;
import frc.robot.Constants.GamepieceManipulator.Elevator.gamepieceHeights;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GPMAutoParallelMidElevatorArmCube extends ParallelCommandGroup {
  /** Creates a new GPMAutoParallelMidElevatorArmCube.
   * Autonomous Parallel part -
   * Elevator UP to CONE heights,
   * Arm out after waiting
   */
  public GPMAutoParallelMidElevatorArmCube() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorToPredefinedHeight(Elevator.gamepieceHeights.AutoCubeMid),
      //new PrintCommand("Elevator Done"),
      new GPMAutoMidArmOutWithWait()
      
    );
  }
}
