// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RaiseElevatorFromStopPin extends CommandBase {

  private double elevatorHeightOffThePin = 0.4; // meters
  private double elevatorHeightOffThePinTolerance = 0.02;
  /** Creates a new RaiseElevatorFromStopPin. */
  public RaiseElevatorFromStopPin() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.elevatorSubsystem.moveToPositionMeters(elevatorHeightOffThePin);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Elevator off the pin at height "+RobotContainer.elevatorSubsystem.getHeight()+ " I:"+interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(RobotContainer.elevatorSubsystem.getHeight() - elevatorHeightOffThePin) <= elevatorHeightOffThePinTolerance ;
  }
}
