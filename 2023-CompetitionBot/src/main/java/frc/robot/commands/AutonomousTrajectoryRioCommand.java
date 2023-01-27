// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class AutonomousTrajectoryRioCommand extends PPRamseteCommand {
  /** Creates a new AutonomousTrajectoryRioCommand. */

  TrajectoryConfig config;

  PathPlannerTrajectory trajectoryPath;
  
  public AutonomousTrajectoryRioCommand(PathPlannerTrajectory trajectoryPath) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
      trajectoryPath,
      RobotContainer.driveSubsystem::getPose,
      new RamseteController(),
      new SimpleMotorFeedforward(
          DriveConstants.ksVolts,
          DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      RobotContainer.driveSubsystem::getWheelSpeeds,
      new PIDController(DriveConstants.trajectoryRioPidP_Value,
        DriveConstants.trajectoryRioPidI_Value,
        DriveConstants.trajectoryRioPidD_Value),
      new PIDController(DriveConstants.trajectoryRioPidP_Value,
        DriveConstants.trajectoryRioPidI_Value,
        DriveConstants.trajectoryRioPidD_Value),
      // RamseteCommand passes volts to the callback
      RobotContainer.driveSubsystem::tankDriveVolts,
      true,
      RobotContainer.driveSubsystem
    );
    this.trajectoryPath = trajectoryPath;
  }

  public AutonomousTrajectoryRioCommand(String trajectoryName, double maxVelocity, double maxAcceleration){
    this(PathPlanner.loadPath(trajectoryName, new PathConstraints(maxVelocity, maxAcceleration)));
    System.out.println("initalized trajectory: "+ trajectoryName + "V:"+maxVelocity+" A:"+maxAcceleration);
  }

  public AutonomousTrajectoryRioCommand(String trajectoryName){
    this(PathPlanner.loadPath(trajectoryName, 
      new PathConstraints(DriveConstants.maxVelocityDefault, DriveConstants.maxAccelerationDefault)));
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
