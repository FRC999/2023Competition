// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.NavigationConstants;
import frc.robot.GPMHelper;

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
    System.out.println("=== RPose:"+RobotContainer.navigationSubsystem.getCurrentPoseOfRobot().toString());

    // TEST - print turret center pose
    Pose2d lt = RobotContainer.navigationSubsystem.calculatePoseOfTurret(RobotContainer.navigationSubsystem.getCurrentPoseOfRobotLeft(),NavigationConstants.leftCameraPose);
    System.out.println("=== TPoseL:"+lt.toString());
    Pose2d rt = RobotContainer.navigationSubsystem.calculatePoseOfTurret(RobotContainer.navigationSubsystem.getCurrentPoseOfRobotRight(),NavigationConstants.rightCameraPose);
    System.out.println("=== TPoseR: "+rt.toString());

    // Nearest AprilTag
    Pose2d pNAT = GPMHelper.identifyNearestTarget(RobotContainer.navigationSubsystem.getCurrentPoseOfRobot());
    System.out.println("NearestAT: "+pNAT.toString());

    // Poses in nearest lane
    Pose2d[] targetPoses = GPMHelper.getTargetPoseFromLaneRecognition(RobotContainer.navigationSubsystem.getCurrentPoseOfRobot(),pNAT);
    System.out.println("GP0: "+targetPoses[0].toString());
    System.out.println("GP1: "+targetPoses[1].toString());
    System.out.println("GP2: "+targetPoses[2].toString());

    // Test GPM positioning
    double[] gpmEX = GPMHelper.getTurretRotationAndArmExtension(RobotContainer.navigationSubsystem.getCurrentPoseOfRobot(),targetPoses[2]);
    System.out.println("T Angle:"+gpmEX[0]+" Arm:"+gpmEX[1]);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.navigationSubsystem.enoughPosesAcquired();
  }
}
