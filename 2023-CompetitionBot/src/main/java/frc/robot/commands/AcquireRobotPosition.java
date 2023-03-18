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

    // TEST - print best LL pose considering both LL
    System.out.println("=== Best LL Pose:"+RobotContainer.navigationSubsystem.getCurrentPoseOfLL().toString());

    // TEST - print turret center pose
    Pose2d lt = RobotContainer.navigationSubsystem.calculatePoseOfTurret(RobotContainer.navigationSubsystem.getCurrentPoseOfLLLeft(),NavigationConstants.leftCameraPose);
    System.out.println("=== TurretCenterUsingLL Left:"+lt.toString());
    Pose2d rt = RobotContainer.navigationSubsystem.calculatePoseOfTurret(RobotContainer.navigationSubsystem.getCurrentPoseOfLLRight(),NavigationConstants.rightCameraPose);
    System.out.println("=== TurretCenterUsingLL Right: "+rt.toString());
    System.out.println("== Pose of LL-Left" + RobotContainer.navigationSubsystem.getCurrentPoseOfLLLeft().toString());
    System.out.println("== Pose of LL-Right" + RobotContainer.navigationSubsystem.getCurrentPoseOfLLRight().toString());

    // Nearest AprilTag
    Pose2d poseOfNearestAprilTag = GPMHelper.identifyNearestTarget(RobotContainer.navigationSubsystem.getCurrentPoseOfLL());
    System.out.println("NearestAprilTagPose: "+poseOfNearestAprilTag.toString());

    // Poses in nearest lane
    Pose2d[] targetPoses = GPMHelper.getTargetPoseFromLaneRecognition(RobotContainer.navigationSubsystem.getCurrentPoseOfLL(),poseOfNearestAprilTag);
    System.out.println("GP0: "+targetPoses[0].toString());
    System.out.println("GP1: "+targetPoses[1].toString());
    System.out.println("GP2: "+targetPoses[2].toString());

    // Test GPM positioning
    double[] gpmEX0 = GPMHelper.getTurretRotationAndArmExtension(RobotContainer.navigationSubsystem.getCurrentPoseOfLL(),targetPoses[0]);
    double[] gpmEX1 = GPMHelper.getTurretRotationAndArmExtension(RobotContainer.navigationSubsystem.getCurrentPoseOfLL(),targetPoses[1]);
    double[] gpmEX2 = GPMHelper.getTurretRotationAndArmExtension(RobotContainer.navigationSubsystem.getCurrentPoseOfLL(),targetPoses[2]);
    System.out.println("T0 Angle:"+gpmEX0[0]+" Arm:"+gpmEX0[1]);
    System.out.println("T1 Angle:"+gpmEX1[0]+" Arm:"+gpmEX0[1]);
    System.out.println("T2 Angle:"+gpmEX2[0]+" Arm:"+gpmEX0[1]);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.navigationSubsystem.enoughPosesAcquired();
  }
}
