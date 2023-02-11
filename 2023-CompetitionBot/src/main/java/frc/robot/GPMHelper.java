// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NavigationConstants;
import frc.robot.Constants.TargetConstants;

public class GPMHelper {
  
  /** Creates a new LaneRecognitionSubsystem. */

  public GPMHelper() {
    
    // Calculate Apriltag target poses for left and right sides of the field
    populateListsOfTargetPoses();
    
  }

  /**
   * Find a pose in the list of Pose2d "poses" that has smallest linear distance 
   * from the "mainPbject" Pose2d
   * @param mainObject
   * @param poses - list of nearest poses to consider
   * @return - nearest Pose2d from the list
   */
  public Pose2d nearest(Pose2d mainObject, List<Pose2d> poses) {
    return Collections.min(
        poses,
        Comparator.comparing(
                (Pose2d other) -> mainObject.getTranslation().getDistance(other.getTranslation()))
            .thenComparing(
                (Pose2d other) ->
                    Math.abs(mainObject.getRotation().minus(other.getRotation()).getRadians())));
  }
  

  /**
   * Consider direction of the Robot pose (left or right)
   * Return direction of the AprilTag target closest to the robotPose in that direction
   * All target poses are stored in Conostants.NavigationConstants
   * @param robotPose - robot Pose2d
   * @return - Pose2d of the nearest AprilTag target
   */
  public Pose2d identifyNearestTarget(Pose2d robotPose) {   
    if(isFacingRight(robotPose)) {
       return nearest(robotPose, NavigationConstants.rightTargets); 
    } else { // Facing LEFT
      return nearest(robotPose, NavigationConstants.leftTargets); 
   }
  }

  /**
   * Determines the 3 poses of the game piece placement given the robot pose and the nearest AprilTag pose
   * 3 poses - low target, middle target, high target
   * Takes into consideration which lane the robot should go to (depending on if facing left or right)
   * @param currentRobotPose
   * @param nearestApriltagPose
   * @return - an array of 3 Pose2d objects 
   */
  public static Pose2d[] getTargetPoseFromLaneRecognition(Pose2d currentRobotPose, Pose2d nearestApriltagPose) {

    if ( currentRobotPose.getY()+ 0.5 * NavigationConstants.yTargetOffset < nearestApriltagPose.getY()) { // I am in a lane BELOW the one with the AprilTag
      if (isFacingLeft(currentRobotPose)) {
        return new Pose2d[] {
          new Pose2d(nearestApriltagPose.getX() + NavigationConstants.xTargetOffset[0], nearestApriltagPose.getY() - NavigationConstants.yTargetOffset, Rotation2d.fromDegrees(180)), // lower target
          new Pose2d(nearestApriltagPose.getX() + NavigationConstants.xTargetOffset[1], nearestApriltagPose.getY() - NavigationConstants.yTargetOffset, Rotation2d.fromDegrees(180)), // middle target
          new Pose2d(nearestApriltagPose.getX() + NavigationConstants.xTargetOffset[2], nearestApriltagPose.getY() - NavigationConstants.yTargetOffset, Rotation2d.fromDegrees(180)) // high target
        }; 
      }else { // Facing RIGHT
        return new Pose2d[] {
          new Pose2d(nearestApriltagPose.getX() - NavigationConstants.xTargetOffset[0], nearestApriltagPose.getY() - NavigationConstants.yTargetOffset, Rotation2d.fromDegrees(0)), //lower target
          new Pose2d(nearestApriltagPose.getX() - NavigationConstants.xTargetOffset[1], nearestApriltagPose.getY() - NavigationConstants.yTargetOffset, Rotation2d.fromDegrees(0)), //middle target
          new Pose2d(nearestApriltagPose.getX() - NavigationConstants.xTargetOffset[2], nearestApriltagPose.getY() - NavigationConstants.yTargetOffset, Rotation2d.fromDegrees(0))  //high target
        };
      }
    } else if (currentRobotPose.getY()- 0.5 * NavigationConstants.yTargetOffset > nearestApriltagPose.getY()) { // I am in a lane ABOVE the one with the AprilTag
      if (isFacingLeft(currentRobotPose)) {
        return new Pose2d[] {
          new Pose2d(nearestApriltagPose.getX() + NavigationConstants.xTargetOffset[0], nearestApriltagPose.getY() + NavigationConstants.yTargetOffset, Rotation2d.fromDegrees(180)), // lower target
          new Pose2d(nearestApriltagPose.getX() + NavigationConstants.xTargetOffset[1], nearestApriltagPose.getY() + NavigationConstants.yTargetOffset, Rotation2d.fromDegrees(180)), // middle target
          new Pose2d(nearestApriltagPose.getX() + NavigationConstants.xTargetOffset[2], nearestApriltagPose.getY() + NavigationConstants.yTargetOffset, Rotation2d.fromDegrees(180)) // high target
        }; 
      } else { // Facing RIGHT
        return new Pose2d[] {
          new Pose2d(nearestApriltagPose.getX() - NavigationConstants.xTargetOffset[0], nearestApriltagPose.getY() + NavigationConstants.yTargetOffset, Rotation2d.fromDegrees(0)), //lower target
          new Pose2d(nearestApriltagPose.getX() - NavigationConstants.xTargetOffset[1], nearestApriltagPose.getY() + NavigationConstants.yTargetOffset, Rotation2d.fromDegrees(0)), //middle target
          new Pose2d(nearestApriltagPose.getX() - NavigationConstants.xTargetOffset[2], nearestApriltagPose.getY() + NavigationConstants.yTargetOffset, Rotation2d.fromDegrees(0))  //high target
        };
      }
    } else { // I am in the same lane where the AprilTag is
      if (isFacingLeft(currentRobotPose)) {
        return new Pose2d[] {
          new Pose2d(nearestApriltagPose.getX() + NavigationConstants.xTargetOffset[0], nearestApriltagPose.getY(), Rotation2d.fromDegrees(180)), // lower target
          new Pose2d(nearestApriltagPose.getX() + NavigationConstants.xTargetOffset[1], nearestApriltagPose.getY(), Rotation2d.fromDegrees(180)), // middle target
          new Pose2d(nearestApriltagPose.getX() + NavigationConstants.xTargetOffset[2], nearestApriltagPose.getY(), Rotation2d.fromDegrees(180)) // high target
        }; 
      } else { // Facing RIGHT
        return new Pose2d[] {
          new Pose2d(nearestApriltagPose.getX() - NavigationConstants.xTargetOffset[0], nearestApriltagPose.getY(), Rotation2d.fromDegrees(0)), //lower target
          new Pose2d(nearestApriltagPose.getX() - NavigationConstants.xTargetOffset[1], nearestApriltagPose.getY(), Rotation2d.fromDegrees(0)), //middle target
          new Pose2d(nearestApriltagPose.getX() - NavigationConstants.xTargetOffset[2], nearestApriltagPose.getY(), Rotation2d.fromDegrees(0))  //high target
        };
      }
    } 
  }


/**
 * Determine if the robot is facing RIGHT
 * @param currentPose - pass current robot pose
 * @return true if robot is facing RIGHT
 */
public static boolean isFacingRight(Pose2d currentPose) {
  return ( ( Math.abs(currentPose.getRotation().getDegrees()) >= 270 && 
  Math.abs(currentPose.getRotation().getDegrees())<=360 )  || 
  Math.abs(currentPose.getRotation().getDegrees())<=90);
 }

 /**
 * Determine if the robot is facing LEFT
 * @param currentPose - pass current robot pose
 * @return true if robot is facing LEFT
 */
 public static boolean isFacingLeft(Pose2d currentPose) {
  return ( ( Math.abs(currentPose.getRotation().getDegrees()) <= 270 && 
  Math.abs(currentPose.getRotation().getDegrees())>=360 )  || 
  Math.abs(currentPose.getRotation().getDegrees())>=90);
 }

 
 /**
  * Populate listr of target poses inn the "tags" array
  * Array index corresponds to the AprilTag Fiduciary ID
  * Left tags have a Rotation of 180, 
  */
 public static void populateListsOfTargetPoses() {
   for (int i = 6; i <= 8; i++) {
     NavigationConstants.leftTargets.add(
         new Pose2d(NavigationConstants.tags[i][0],
             NavigationConstants.tags[i][1],
             new Rotation2d(Math.toRadians(180))));
   }
   for (int i = 1; i <= 3; i++) {
     NavigationConstants.rightTargets.add(
         new Pose2d(NavigationConstants.tags[i][0],
             NavigationConstants.tags[i][1],
             new Rotation2d(Math.toRadians(0))));
   }
 }

  
  

  
}


