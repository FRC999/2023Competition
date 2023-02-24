// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GPMHelper;
import frc.robot.RobotContainer;
import frc.robot.Constants.NavigationConstants;

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

  public boolean enoughPosesAcquired() {
    if (numberOfValidMeasurementsLeft >= NavigationConstants.numberOfMeasurements ||
      numberOfValidMeasurementsRight >= NavigationConstants.numberOfMeasurements || 
      numberOfTotalMeasurements >= NavigationConstants.numberOfMaxPoseMeasurements ) {

        // TEST - number of poses acquired
        System.out.println("Poses L:"+limelightPoseManagerLeft.numberOfPoses()+" R:"+limelightPoseManagerRight.numberOfPoses());

      return true;
    }
    return false;
  }

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
