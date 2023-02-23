// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GPMHelper;
import frc.robot.RobotContainer;
import frc.robot.Constants.NavigationConstants;

public class NavigationSubsystem extends SubsystemBase {

  private boolean isCollectingPoses = false;
  private int[] numberOfValidMeasurements = new int[2];
  private PoseManager[] limelightPoseManager = new PoseManager[2];

  /** Creates a new NavigationSubsystem. */
  Pose2d currentAngleFromPose;

  public NavigationSubsystem() {
    currentAngleFromPose = getCurrentPoseOfRobot();
    GPMHelper.populateListsOfTargetPoses();

  }

  // Start acquisition of the robot poses
  public void acquireRobotPoses() {
    numberOfValidMeasurements[0] = 0;
    numberOfValidMeasurements[1] = 0;
    limelightPoseManager[0] = new PoseManager();
    limelightPoseManager[1] = new PoseManager();
    isCollectingPoses = true;
  }

  public boolean enoughPosesAcquired() {
    if (numberOfValidMeasurements[0] >= NavigationConstants.numberOfMeasurements ||
      numberOfValidMeasurements[1] >= NavigationConstants.numberOfMeasurements ) {

        // TEST - number of poses acquired
        System.out.println("Poses L:"+limelightPoseManager[0].numberOfPoses()+" R:"+limelightPoseManager[0].numberOfPoses());

      return true;
    }
    return false;
  }

  public Pose2d getCurrentPoseOfRobot() {
    return null; //TODO: change this later
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (isCollectingPoses) {
      if (RobotContainer.networkTablesSubsystem.isLeftTargetAcquired()) {
        limelightPoseManager[0].addPose(RobotContainer.networkTablesSubsystem.getLimelightLeftRobotPose());
        numberOfValidMeasurements[0]++;
      }
      if (RobotContainer.networkTablesSubsystem.isRightTargetAcquired()) {
        limelightPoseManager[1].addPose(RobotContainer.networkTablesSubsystem.getLimelightRightRobotPose());
        numberOfValidMeasurements[1]++;
      }

      // Stop pose acquisition if enough poses are acquired
      if (enoughPosesAcquired()) {
        isCollectingPoses =  false;
      }

    }
  }
}
