// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GPMHelper;
import frc.robot.RobotContainer;
import frc.robot.Constants.NavigationConstants;
import frc.robot.Constants.GamepieceManipulator.Arm;

public class NavigationSubsystem extends SubsystemBase {

  private boolean isCollectingPoses = false;
  private int numberOfValidMeasurementsLeft;
  private int numberOfValidMeasurementsRight;
  private int numberOfTotalMeasurements;
  private PoseManager limelightPoseManagerLeft = new PoseManager();
  private PoseManager limelightPoseManagerRight = new PoseManager();

  // false - left camera; true - right camera
  private final Rotation2d turretToCameraAngleLeft = 
    new Rotation2d(NavigationConstants.leftCameraPose.getX(), NavigationConstants.leftCameraPose.getY());

  private final Rotation2d turretToCameraAngleRight = 
    new Rotation2d(NavigationConstants.rightCameraPose.getX(), NavigationConstants.rightCameraPose.getY());

  /** Creates a new NavigationSubsystem. */
  Pose2d currentAngleFromPose;

  public NavigationSubsystem() {
    currentAngleFromPose = getCurrentPoseOfLL();
    GPMHelper.populateListsOfTargetPoses();
  }

  // Start acquisition of the robot poses
  public void acquireRobotPoses() {
    numberOfValidMeasurementsLeft = 0;
    numberOfValidMeasurementsRight = 0;
    numberOfTotalMeasurements = 0;
    limelightPoseManagerLeft.clearAllPoses();
    limelightPoseManagerRight.clearAllPoses();
    isCollectingPoses = true;
  }

  /**
   * Check if we acquired enough pose samples
   * Stop if left or right camera got enough measurements
   * or if we exceed the maximum number of measurements we want to try
   * @return
   */
  public boolean enoughPosesAcquired() {
    if (numberOfValidMeasurementsLeft >= NavigationConstants.numberOfMeasurements ||
      numberOfValidMeasurementsRight >= NavigationConstants.numberOfMeasurements || 
      numberOfTotalMeasurements >= NavigationConstants.numberOfMaxPoseMeasurements ) {

        // TEST - number of poses acquired
        //System.out.println("Poses L:"+limelightPoseManagerLeft.numberOfPoses()+" R:"+limelightPoseManagerRight.numberOfPoses());

      return true;
    }
    return false;
  }

  /**
   * Calculate pose of the camera with relative trust
   * Give more trust to the camera that collected more samples
   * If the same number of samples, use left camera
   * @return - pose of the camera
   */
  public Pose2d getCurrentPoseOfLL() {
    if (! isCollectingPoses) {
      if (limelightPoseManagerLeft.numberOfPoses()>=limelightPoseManagerRight.numberOfPoses()) {
        return limelightPoseManagerLeft.getPose();
      } else {
        return limelightPoseManagerRight.getPose();
      }
    }
    return NavigationConstants.dummyPose; // return dummy pose if collecting poses right now
  }

  // In case we only want the pose calculated for left camera
  public Pose2d getCurrentPoseOfLLLeft() {
    if (! isCollectingPoses) {
        return limelightPoseManagerLeft.getPose();
    }
    return NavigationConstants.dummyPose; // return dummy pose if collecting poses right now
  }
  // In case we only want the pose calculated for right camera
  public Pose2d getCurrentPoseOfLLRight() {
    if (! isCollectingPoses) {
        return limelightPoseManagerRight.getPose();
    }
    return NavigationConstants.dummyPose; // return dummy pose if collecting poses right now
  }

  /**
   * Recalculate angle in 0-360 range relative to another angle
   * @param zeroAngle - "stamdard" angle that is the base we're calculating agains
   * @param rawPoseAngle - "measured" angle to be recalculated
   * @return - degrees 0-360
   */
  public double recalculateAngle(double zeroAngle, double rawPoseAngle){

    System.out.println("ZA:"+zeroAngle+" RA:"+rawPoseAngle+" TA:"+((rawPoseAngle - zeroAngle  + 360)%360));

    return ((rawPoseAngle - zeroAngle  + 360)%360);
   }

  /**
   * Calculate the pose of the turret center based on the pose of the camera on the field
   * as well as relative pose of the camera to the center of the turret
   * @param locationOfCamera  - location of the camera on the field
   * @param zeroPoseofCamera - pose of the camera relative to the center of the turret (offsets); stored in NavigationConstants
   * @return - pose of the center of the turret
   */
  public Pose2d calculatePoseOfTurret(Pose2d locationOfCamera, Pose2d zeroPoseofCamera){

    double turretDistFromCenterToCameraLens = 
       Math.sqrt(Math.pow(zeroPoseofCamera.getX(), 2) + Math.pow(zeroPoseofCamera.getY(), 2));

    // Angle from 0-direction of the turret to the camera
    double turretAngle = recalculateAngle(zeroPoseofCamera.getRotation().getDegrees(), 
      locationOfCamera.getRotation().getDegrees());

    double turretLookingAtCameraAngle = turretAngle + new Rotation2d(zeroPoseofCamera.getX(), zeroPoseofCamera.getY()).getDegrees();

    System.out.println("TA:"+turretAngle + " T-LA:"+turretLookingAtCameraAngle);
  
    double currentTurretX = locationOfCamera.getX()
      - Math.cos(Units.degreesToRadians(turretLookingAtCameraAngle)) *
      turretDistFromCenterToCameraLens;
  
    double currentTurretY = locationOfCamera.getY()
      - Math.sin(Units.degreesToRadians(turretLookingAtCameraAngle)) *
      turretDistFromCenterToCameraLens;
  
    return new Pose2d(currentTurretX, currentTurretY, new Rotation2d(Units.degreesToRadians(turretAngle)));
    
  }

  /**
   * After getting LL position from cameras, calculate robot-centric turret angle to place a gamepiece 
   * @param position 0-low, 1-mid, 2-high
   * @param movement 0-turret angle, 1-arm length
   * @return {turret_angle_degrees,arm_length_meters}
   */
  public double getTurretArmToTarget(int position, int movement) {

    if (MathUtil.clamp(position,0,2) != position) {
      System.out.println("P-F"+MathUtil.clamp(position,0,2));
      return Double.NaN;
    }

    if (MathUtil.clamp(movement,0,1) != movement) {
      System.out.println("M-F"+MathUtil.clamp(movement,0,1));
      return Double.NaN;
    }

    // test - comment out
    System.out.println("Clamp passed");

    // Current LL pose
    Pose2d llPose = RobotContainer.navigationSubsystem.getCurrentPoseOfLL() ;

    // Nearest AprilTag pose
    // Pose2d poseOfNearestAprilTag = GPMHelper.identifyNearestTarget(llPose);

    // Poses of targets in the lane neares to me
    Pose2d[] targetPoses = GPMHelper.getTargetPoseFromLaneRecognition(llPose,GPMHelper.identifyNearestTarget(llPose));

    // 0-element - rotation angle, 1-element - length
    return ( GPMHelper.getTurretRotationAndArmExtension(llPose,targetPoses[position]) )[movement];

  }

  public void printTurretArmToTarget(int position) {
    System.out.println("TurretAngle:"+ getTurretArmToTarget(position, 0));
    System.out.println("ArmLength:"+getTurretArmToTarget(position, 1));
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (isCollectingPoses) {
      if (RobotContainer.networkTablesSubsystem.isLeftTargetAcquired()) {
        limelightPoseManagerLeft.addPose(RobotContainer.networkTablesSubsystem.getLimelightLeftRobotPose());
        numberOfValidMeasurementsLeft++;
      }
      if (RobotContainer.networkTablesSubsystem.isRightTargetAcquired()) {
        limelightPoseManagerRight.addPose(RobotContainer.networkTablesSubsystem.getLimelightRightRobotPose());
        numberOfValidMeasurementsRight++;
      }

      numberOfTotalMeasurements++;

      // Stop pose acquisition if enough poses are acquired
      if (enoughPosesAcquired()) {
        isCollectingPoses =  false;
      }

    }
  }
}
