// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.NavigationConstants;

import frc.robot.Constants.GamepieceManipulator.Arm;

public class GPMHelper {
  
  /** Creates a new LaneRecognitionSubsystem. */

  //public GPMHelper() {
    
    // Calculate Apriltag target poses for left and right sides of the field
  //  populateListsOfTargetPoses();
    
  //}

  /**
   * Find a pose in the list of Pose2d "poses" that has smallest linear distance 
   * from the "mainPbject" Pose2d
   * @param mainObject
   * @param poses - list of nearest poses to consider
   * @return - nearest Pose2d from the list
   */
  public static Pose2d nearest(Pose2d mainObject, List<Pose2d> poses) {
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
  public static Pose2d identifyNearestTarget(Pose2d robotPose) {   
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
          new Pose2d(nearestApriltagPose.getX() + NavigationConstants.xTargetOffset[0], nearestApriltagPose.getY() - NavigationConstants.yTargetOffset, Rotation2d.fromDegrees(180)), // lower targetRotation2d
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
   * Get Turret rotation angle and arm extension length for known robot Pose2d and target Pose2d
   * If the target is impossible to reach (arm is not long enough), return -1 for the length
   * The angle is calculated from 0 direction of the robot (front)
   * @param currentTurretPose - Pose2d of the robot
   * @param targetPose - Pose2d where I want to place a target
   * @return - double[] : 0 - turret rotation angle (degrees); 1 - arm extension (meters)
   */
  public static double[] getTurretRotationAndArmExtension(Pose2d currentTurretPose, Pose2d targetPose) {

    // convert turret into the robot-centric angle

    currentTurretPose = new Pose2d(currentTurretPose.getTranslation(),
        new Rotation2d(currentTurretPose.getRotation().getRadians() 
          - Units.degreesToRadians(RobotContainer.turretSubsystem.getDegrees()))
        );

    // This calculates linear distance between two poses
    // Note that simple scalar distances are calculated between Translation2d, hence it's extracted from Pose2d first
    double armExtension = currentTurretPose.getTranslation().getDistance(targetPose.getTranslation());

    // Angle calculcation between two poses
    // May be optimized considering current location of the Arm
    // The reason variable was declared above, so not ot use expensive getTranslation and getDistance multiple times
    // We assume that angle from each pose is between -360..+360

    //double angleDiff = (targetPose.getRotation().getDegrees() - currentRobotPose.getRotation().getDegrees() + 360.0) % 360 ;
    //double angleDiff = Units.radiansToDegrees(currentRobotPose.log(targetPose).dtheta);

    // This constructs a Rotation2d object based on a vector pointing from acurrentRobotPose to targetPose
    Rotation2d pointerToTarget = new Rotation2d(targetPose.getX()-currentTurretPose.getX(),targetPose.getY()-currentTurretPose.getY());

    // The difference between two rotations gives you an angle
    double angleDiff = pointerToTarget.minus(currentTurretPose.getRotation()).getDegrees();

    // If angleDiff is positive, calculate correspponding negative angle, and vice versa

    double secondAngleDiff = (angleDiff<=0)?angleDiff+360:angleDiff-360;

    // In a first-cut optimization method we will use the angle with the lowest absolute value
    // If Turret currently is at 0, that will determine the minimum rotation
    // and will never exceed the turret limit, providing that the limit is above +-180
    // For the turret the angle will mean relative rotation from 0 that is in front of the robot

    return new double[] {
      (Math.abs(angleDiff)<Math.abs(secondAngleDiff))?angleDiff:secondAngleDiff,
      (armExtension > Arm.maximumExtension)?(-1):armExtension // If arm extension exceeeds maximum possible, return -1
    };

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
             new Rotation2d(Units.degreesToRadians(180))));
   }
   for (int i = 1; i <= 3; i++) {
     NavigationConstants.rightTargets.add(
         new Pose2d(NavigationConstants.tags[i][0],
             NavigationConstants.tags[i][1],
             new Rotation2d(Units.degreesToRadians(0))));
   }
 }
}


