// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamepieceManipulator.Elevator;

public class DriveElevatorManuallyCommand extends CommandBase {
  /** Creates a new DriveElevatorManuallyCommand. */
  public DriveElevatorManuallyCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double power = -RobotContainer.gpmStick.getY();
    if (power<0) {
      if (RobotContainer.elevatorSubsystem.getEncoder()<=Elevator.elevatorAbsoluteZero) {
        power=0;
      }
    } else {
      if (RobotContainer.elevatorSubsystem.getEncoder()>=Elevator.elevatorMaxLimit) {
        power=0;
      }
    }
    RobotContainer.elevatorSubsystem.manualDrive(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = -RobotContainer.gpmStick.getY();
    System.out.print("power: " + power);
    if (power<0) {
      if (RobotContainer.elevatorSubsystem.getEncoder()<=0) {
        power=0;
      }
    } else {
      if (RobotContainer.elevatorSubsystem.getEncoder()>=Elevator.elevatorMaxLimit) {
        power=0;
      }
    }
    RobotContainer.elevatorSubsystem.manualDrive(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
