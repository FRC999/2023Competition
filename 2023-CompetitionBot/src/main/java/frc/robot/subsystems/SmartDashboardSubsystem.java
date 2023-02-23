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

  public void updateOdometryTelemetry() {
    SmartDashboard.putNumber("OdometryX", RobotContainer.driveSubsystem.returnOdometry().getPoseMeters().getX());
    SmartDashboard.putNumber("OdometryY", RobotContainer.driveSubsystem.returnOdometry().getPoseMeters().getY());
  }

  public void updateRobotPose() {

    SmartDashboard.putNumber("LL1-X", RobotContainer.networkTablesSubsystem.getLimelightLeftRobotPose().getX());
    SmartDashboard.putNumber("LL1-Y", RobotContainer.networkTablesSubsystem.getLimelightLeftRobotPose().getY());
    SmartDashboard.putNumber("LL1-angle", RobotContainer.networkTablesSubsystem.getLimelightLeftRobotPose().getRotation().getDegrees());
    SmartDashboard.putBoolean("LL1-visible", RobotContainer.networkTablesSubsystem.isLeftTargetAcquired());

    SmartDashboard.putNumber("LL2-X", RobotContainer.networkTablesSubsystem.getLimelightRightRobotPose().getX());
    SmartDashboard.putNumber("LL2-Y", RobotContainer.networkTablesSubsystem.getLimelightRightRobotPose().getY());
    SmartDashboard.putNumber("LL2-angle", RobotContainer.networkTablesSubsystem.getLimelightRightRobotPose().getRotation().getDegrees());
    SmartDashboard.putBoolean("LL2-visible", RobotContainer.networkTablesSubsystem.isRightTargetAcquired());

  }
  
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

  public void updateIMUTelemetry() {
    SmartDashboard.putNumber("IMU Yaw", RobotContainer.pigeonIMUSubsystem.getYaw());
    SmartDashboard.putNumber("IMU Pitch", RobotContainer.pigeonIMUSubsystem.getPitch());
    SmartDashboard.putNumber("IMU Roll", RobotContainer.pigeonIMUSubsystem.getRoll());
    SmartDashboard.putNumber("IMU Heading", RobotContainer.pigeonIMUSubsystem.getHeading());
    SmartDashboard.putNumber("IMU TurnRate", RobotContainer.pigeonIMUSubsystem.getTurnRate());
  }


  public void updateAllDisplays() {

    //updateDriveSubsystemTelemetry();
    //updateIMUTelemetry();
    updateRobotPose();
    //updateOdometryTelemetry();
    //updateTurretTelemetry();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateAllDisplays();
  }
}
