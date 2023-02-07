// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.NavigationConstants;

public class LaneRecognitionSubsystem extends SubsystemBase {
  /** Creates a new LaneRecognitionSubsystem. */

  public LaneRecognitionSubsystem() {
    
    // Calculate Apriltag target poses for left and right sides of the field
    populateListsOfTargetPoses();
    
  }
//use isFacingRight, and getcurrentPoseOfRobot
  public void identifyNearestTarget() {   //what return type would we want this 2 be
    if(RobotContainer.navigationSubsystem.isFacingRight()) {
      
    }
  }

  public void populateListsOfTargetPoses() {
    for(int i=6; i<=8; i++){
      NavigationConstants.leftTargets.add(
          new Pose2d(NavigationConstants.tags[i][0],
              NavigationConstants.tags[i][1],
              new Rotation2d(Math.toRadians(180))));
    }
    for(int i=1; i<=3; i++){
      NavigationConstants.rightTargets.add(
          new Pose2d(NavigationConstants.tags[i][0],
              NavigationConstants.tags[i][1],
              new Rotation2d(Math.toRadians(0))));
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


