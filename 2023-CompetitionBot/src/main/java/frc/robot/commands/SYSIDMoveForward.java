// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SYSIDMoveForward extends SequentialCommandGroup {
  /** Creates a new SYSIDMoveForward. */

  PathPlannerTrajectory trajectoryPath;
  
  public SYSIDMoveForward(String trajectory, double maxVelocity, double maxAcceleration) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand( () -> RobotContainer.driveSubsystem.resetOdometry(trajectoryPath.getInitialPose()) ),  // Set the initial pose of the robot to the one in a trajectory
      new AutonomousTrajectoryRioCommand(trajectoryPath) // Run a trajectory
    );
  }

  public SYSIDMoveForward(String trajectory) {

    this(trajectory, DriveConstants.maxVelocityDefault, DriveConstants.maxAccelerationDefault);
    System.out.println("*** Run trajectory "+ trajectory);
  }

  //Called when the command is initially scheduled.
  
}
