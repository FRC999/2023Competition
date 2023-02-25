// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.NavigationConstants;

public class AcquireRobotPosition extends CommandBase {
  /** Creates a new AcquireRobotPosition. */
  public AcquireRobotPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.navigationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.navigationSubsystem.acquireRobotPoses();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("*** Robot pose samples acquired. I:"+interrupted);

    // TEST - print robot pose
    System.out.println("=== RPose:"+RobotContainer.navigationSubsystem.getCurrentPoseOfRobot().getX()+
      ","+RobotContainer.navigationSubsystem.getCurrentPoseOfRobot().getY()+","+
      RobotContainer.navigationSubsystem.getCurrentPoseOfRobot().getRotation().getDegrees());

    // TEST - print turret center pose
    Pose2d lt = RobotContainer.navigationSubsystem.calculatePoseOfTurret(RobotContainer.navigationSubsystem.getCurrentPoseOfRobotLeft(),NavigationConstants.leftCameraPose);
    System.out.println("=== TPoseL:"+lt.getX()+
      ","+lt.getY()+","+
      lt.getRotation().getDegrees());
    Pose2d rt = RobotContainer.navigationSubsystem.calculatePoseOfTurret(RobotContainer.navigationSubsystem.getCurrentPoseOfRobotRight(),NavigationConstants.rightCameraPose);
    System.out.println("=== TPoseR:"+rt.getX()+
      ","+rt.getY()+","+
      rt.getRotation().getDegrees());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.navigationSubsystem.enoughPosesAcquired();
  }
}
