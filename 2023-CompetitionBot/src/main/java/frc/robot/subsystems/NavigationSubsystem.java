// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  /** Creates a new NavigationSubsystem. */
  Pose2d currentAngleFromPose;

  public NavigationSubsystem() {
    currentAngleFromPose = getCurrentPoseOfRobot();
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
  public Pose2d getCurrentPoseOfRobot() {
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
  public Pose2d getCurrentPoseOfRobotLeft() {
    if (! isCollectingPoses) {
        return limelightPoseManagerLeft.getPose();
    }
    return NavigationConstants.dummyPose; // return dummy pose if collecting poses right now
  }
  // In case we only want the pose calculated for right camera
  public Pose2d getCurrentPoseOfRobotRight() {
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

    // System.out.println("ZA:"+zeroAngle+" RA:"+rawPoseAngle);

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
  
    double trueAngle = recalculateAngle(zeroPoseofCamera.getRotation().getDegrees(), 
      locationOfCamera.getRotation().getDegrees());

    // System.out.println("TA:"+trueAngle);
  
    double currentTurretX = locationOfCamera.getX()
      - Math.cos(trueAngle) *
      turretDistFromCenterToCameraLens;
  
    double currentTurretY = locationOfCamera.getY()
      - Math.sin(trueAngle) *
      turretDistFromCenterToCameraLens;
  
    return new Pose2d(currentTurretX, currentTurretY, new Rotation2d(Units.degreesToRadians(trueAngle)));
    
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
