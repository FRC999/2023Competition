// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamepieceManipulator.Elevator.coneHeights;

public class ConeHeightCommand extends CommandBase {
  /** Creates a new LowConeHeight. */

  public ConeHeightCommand() {
    addRequirements(RobotContainer.elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.elevatorSubsystem.initializeElevator();

    coneHeights myVar = coneHeights.MidCone;

    switch(myVar) {
      case LowCone:
        
        break;
      case MidCone://Sam Esha not finished will continue on Monday
         // test
        break;
      case HighCone:
        
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 
    Math.abs(RobotContainer.driveSubsystem.getLeftEncoder() - RobotContainer.elevatorSubsystem.clicksPerFullRotationSRX*120) <= Constants.GamepieceManipulator.Elevator.elevator_PIDTolerance ||
    Math.abs(RobotContainer.driveSubsystem.getRightEncoder() - RobotContainer.elevatorSubsystem.clicksPerFullRotationSRX*120) <= Constants.GamepieceManipulator.Elevator.elevator_PIDTolerance;
  }
}
