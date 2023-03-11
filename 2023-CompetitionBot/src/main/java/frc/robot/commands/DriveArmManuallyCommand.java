// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamepieceManipulator.Arm;

public class DriveArmManuallyCommand extends CommandBase {
  /** Creates a new DriveArmManually. */
  public DriveArmManuallyCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double power = -RobotContainer.gpmStick.getRawAxis(3);
    if (power<0) {
      if (RobotContainer.armSubsystem.getEncoder()<=0) {
        power=0;
      }
    } else {
      if (RobotContainer.armSubsystem.getEncoder()>=Arm.armMaxLimit) {
        power=0;
      }
    }
    RobotContainer.armSubsystem.manualDrive(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = -RobotContainer.gpmStick.getRawAxis(3);

    //System.out.println("R3:"+RobotContainer.gpmStick.getRawAxis(3));

    //System.out.println("Z:"+power);

    if (power<0) {
      if (RobotContainer.armSubsystem.getEncoder()<=0) {
        if (power < Arm.armHoldingPower)
        power=Arm.armHoldingPower;  // apply constant force when reached 0 to avoid pulsing
      }
    } else {
      if (RobotContainer.armSubsystem.getEncoder()>=Arm.armMaxLimit  && ! RobotContainer.armSubsystem.getLimitOverride()) {
        power=0;
      }
    }
    RobotContainer.armSubsystem.manualDrive(power + Arm.armHoldingPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("End manual arm drive I:"+interrupted);
    RobotContainer.armSubsystem.armForceFeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
