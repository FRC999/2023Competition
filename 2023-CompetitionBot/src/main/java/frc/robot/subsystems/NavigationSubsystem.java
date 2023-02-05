// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class NavigationSubsystem extends SubsystemBase {
  /** Creates a new NavigationSubsystem. */
  Pose2d currentAngleFromPose;

  public NavigationSubsystem() {
    currentAngleFromPose = getCurrentPoseOfRobot();

  }

  public Pose2d getCurrentPoseOfRobot() {
    return null; //TODO: change this later
    
  }

  public boolean isFacingRight() {
    return ((-90 < currentAngleFromPose.getRotation().getDegrees() && 0 > currentAngleFromPose.getRotation().getDegrees()) 
    || (270 < currentAngleFromPose.getRotation().getDegrees() && 360 > currentAngleFromPose.getRotation().getDegrees())
    || (0 < currentAngleFromPose.getRotation().getDegrees() && 90 > currentAngleFromPose.getRotation().getDegrees())
    || (-270 < currentAngleFromPose.getRotation().getDegrees() && 0 > currentAngleFromPose.getRotation().getDegrees()));
  }
  

  //public

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
