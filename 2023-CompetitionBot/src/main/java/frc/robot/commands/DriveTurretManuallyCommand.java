// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamepieceManipulator.Turret;

public class DriveTurretManuallyCommand extends CommandBase {
  /** Creates a new DriveTurretManuallyCommand. */
  public DriveTurretManuallyCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double power = -RobotContainer.gpmStick.getX();
    if (power<0) {
      if (RobotContainer.turretSubsystem.getEncoder()<=Turret.turretLeftLimit) {
        power=0;
      }
    } else {
      if (RobotContainer.turretSubsystem.getEncoder()>=Turret.turretRightLimit) {
        power=0;
      }
    }
    RobotContainer.turretSubsystem.manualDrive(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = -RobotContainer.gpmStick.getX();
    if (power<0) {
      if (RobotContainer.turretSubsystem.getEncoder()<=Turret.turretLeftLimit) {
        power=0;
      }
    } else {
      if (RobotContainer.turretSubsystem.getEncoder()>=Turret.turretRightLimit) {
        power=0;
      }
    }
    RobotContainer.turretSubsystem.manualDrive(power);
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
