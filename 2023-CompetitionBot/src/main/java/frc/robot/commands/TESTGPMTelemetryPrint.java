// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.GPMHelper;
import frc.robot.RobotContainer;
import frc.robot.Constants.NavigationConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TESTGPMTelemetryPrint extends InstantCommand {
  /**
   * Print GPM telemetry for the current turret position
   */
  public TESTGPMTelemetryPrint() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    System.out.println("GP-Low: "+targetPoses[0].toString());
    System.out.println("GP-Mid: "+targetPoses[1].toString());
    System.out.println("GP-High: "+targetPoses[2].toString());

    // Test GPM positioning
    double[] gpmEXLow = GPMHelper.getTurretRotationAndArmExtension(RobotContainer.navigationSubsystem.getCurrentPoseOfLL(),targetPoses[0]);
    double[] gpmEXMid = GPMHelper.getTurretRotationAndArmExtension(RobotContainer.navigationSubsystem.getCurrentPoseOfLL(),targetPoses[1]);
    double[] gpmEXHigh = GPMHelper.getTurretRotationAndArmExtension(RobotContainer.navigationSubsystem.getCurrentPoseOfLL(),targetPoses[2]);
    System.out.println("T-Low Angle:"+gpmEXLow[0]+" Arm:"+gpmEXLow[1]);
    System.out.println("T-Mid Angle:"+gpmEXMid[0]+" Arm:"+gpmEXMid[1]);
    System.out.println("T-High Angle:"+gpmEXHigh[0]+" Arm:"+gpmEXHigh[1]);
  }
}
