// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamepieceManipulator.Elevator.gamepieceHeights;

public class ElevatorToPredefinedHeight extends CommandBase {

  private static double heightTolerance = 0.02; // meters
  private gamepieceHeights targetHeight;

  /** Set elevator to a predefined height defined in the gamepieceHeights enum */
  public ElevatorToPredefinedHeight(gamepieceHeights goToHeight) {
    addRequirements(RobotContainer.elevatorSubsystem);
    targetHeight = goToHeight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.elevatorSubsystem.moveToPositionMeters(targetHeight.getHeight());
    System.out.println("Elevator moving to height "+targetHeight.getHeight());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Reached target height "+targetHeight.name()+" I:"+interrupted);

    // For GPM Auto gamepiece placement
    if (interrupted) {
      RobotContainer.elevatorSubsystem.clearMiddleCommandStarted();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.elevatorSubsystem.getHeight() - targetHeight.getHeight()) <= heightTolerance ;
  }
}
