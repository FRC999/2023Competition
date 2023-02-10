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
import frc.robot.RobotContainer;
import frc.robot.Constants.NavigationConstants;
import frc.robot.Constants.TargetConstants;

public class LaneRecognitionSubsystem extends SubsystemBase {
  
  /** Creates a new LaneRecognitionSubsystem. */

  public LaneRecognitionSubsystem() {
    
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




  public static int[][] getTargetPoseFromLaneRecognition(int[] currentRobotPose, int[] apriltagPose) {

    int yr = currentRobotPose[1];
    int yt = apriltagPose[1];
    int x = currentRobotPose[0];
    int a = 0; //difference in y coord 
    int b = 0; // b,c,d are difference in x coord
    int c = 0; 
    int d = 0;
    int[][] targetPose;

    
    if (yr + 0.5 * a > yr && yt - 0.5 * a < yr) {
        targetPose = new int[][] {{x - b, yt}, {x - c, yt}, {x - d, yt}}; //for facing left
    } else if (yt - 0.5 * a > yr) {
        targetPose = new int[][] {{x - b, yt - a}, {x - c, yt - a}, {x - d, yt - a}}; //for facing right
    } else {
        targetPose = new int[][] {{x - b, yt + a}, {x - c, yt + a}, {x - d, yt + a}};
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

  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


