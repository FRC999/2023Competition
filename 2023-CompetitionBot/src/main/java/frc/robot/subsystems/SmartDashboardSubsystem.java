// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {}

  public void updateDriveSubsystemTelemetry() {
    SmartDashboard.putNumber("Left Encoder Value", RobotContainer.driveSubsystem.getLeftEncoder());
    SmartDashboard.putNumber("Left Encoder Speed", RobotContainer.driveSubsystem.getLeftEncoderSpeed());
    SmartDashboard.putNumber("Right Encoder Value", RobotContainer.driveSubsystem.getRightEncoder());
    SmartDashboard.putNumber("Right Encoder Speed", RobotContainer.driveSubsystem.getRightEncoderSpeed());
  }

  public void updateTurretTelemetry() {
    SmartDashboard.putNumber("Turret Relative Encoder Value", RobotContainer.turretSubsystem.getEncoder());
    SmartDashboard.putNumber("Turret Absolute Encoder Value", RobotContainer.turretSubsystem.getAbsEncoder());
    SmartDashboard.putNumber("Turret Speed", RobotContainer.turretSubsystem.getSpeed());
    SmartDashboard.putNumber("Turret PID Error", RobotContainer.turretSubsystem.getError());
  }


  public void updateAllDisplays() {

    updateDriveSubsystemTelemetry();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAllDisplays();
  }
}
