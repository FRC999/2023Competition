// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.NavigationConstants;
import frc.robot.Constants.TargetConstants;

public class GPMHelper {
  
  /** Creates a new LaneRecognitionSubsystem. */

  public GPMHelper() {
    
    // Calculate Apriltag target poses for left and right sides of the field
    populateListsOfTargetPoses();
    
  }

  public Pose2d nearest(Pose2d mainObject, List<Pose2d> poses) {
    return Collections.min(
        poses,
        Comparator.comparing(
                (Pose2d other) -> mainObject.getTranslation().getDistance(other.getTranslation()))
            .thenComparing(
                (Pose2d other) ->
                    Math.abs(mainObject.getRotation().minus(other.getRotation()).getRadians())));
  }
  

  public Pose2d identifyNearestTarget(Pose2d detectedPose2dofRobot) {   

    if(RobotContainer.navigationSubsystem.isFacingRight()) {
       //RobotContainer.navigationSubsystem.getCurrentPoseOfRobot();
       //detectedPose2dofRobot.nearest(NavigationConstants.rightTargets);
       return nearest(detectedPose2dofRobot, NavigationConstants.rightTargets); 
    }

    else if(RobotContainer.navigationSubsystem.isFacingLeft()) {
      return nearest(detectedPose2dofRobot, NavigationConstants.leftTargets); 
   }
   return detectedPose2dofRobot;
  }




  public double[][] getTargetPoseFromLaneRecognition(double[] currentRobotPose, double[] apriltagPose) {

    double robotPoseY = currentRobotPose[1];//TODO: will change later, needs to be updated
    double robotPoseX = currentRobotPose[0];//TODO: will change later, needs to be updated
    double targetPoseY = apriltagPose[1];//TODO: will change later, needs to be updated

    double[][] targetPose; //still need to fix targetPose and apriltagPose, respectively


    if (robotPoseY + 0.5 * Constants.NavigationConstants.yTargetOffset > robotPoseY && targetPoseY - 0.5 * Constants.NavigationConstants.yTargetOffset < robotPoseY) {
        targetPose = new double[][] {{robotPoseX - Constants.NavigationConstants.xTargetOffset[0], targetPoseY}, {robotPoseX - Constants.NavigationConstants.xTargetOffset[1], targetPoseY}, {robotPoseX - Constants.NavigationConstants.xTargetOffset[2], targetPoseY}}; //for facing left
    } else if (targetPoseY - 0.5 * Constants.NavigationConstants.yTargetOffset > robotPoseY) {
        targetPose = new double[][] {{robotPoseX - Constants.NavigationConstants.xTargetOffset[0], targetPoseY - Constants.NavigationConstants.yTargetOffset}, {robotPoseX - Constants.NavigationConstants.xTargetOffset[1], targetPoseY - Constants.NavigationConstants.yTargetOffset}, {robotPoseX - Constants.NavigationConstants.xTargetOffset[2], targetPoseY - Constants.NavigationConstants.yTargetOffset}}; //for facing right
    } else {
        targetPose = new double[][] {{robotPoseX - Constants.NavigationConstants.xTargetOffset[0], targetPoseY + Constants.NavigationConstants.yTargetOffset}, {robotPoseX - Constants.NavigationConstants.xTargetOffset[1], targetPoseY + Constants.NavigationConstants.yTargetOffset}, {robotPoseX - Constants.NavigationConstants.xTargetOffset[2], targetPoseY + Constants.NavigationConstants.yTargetOffset}};
    }
    return targetPose;
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

  
  

  
}


