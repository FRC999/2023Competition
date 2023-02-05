// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LaneRecognitionSubsystem extends SubsystemBase {
  /** Creates a new LaneRecognitionSubsystem. */

  public LaneRecognitionSubsystem() {
    
  }
//use isFacingRight, and getcurrentPoseOfRobot
  public void identifyNearestTarget() {   //what return type would we want this 2 be
    if(RobotContainer.navigationSubsystem.isFacingRight()) {
      
    }
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


