// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunTrajectorySequenceRobotAtStartPoint extends SequentialCommandGroup {
  /** Creates a new RunTrajectorySequenceRobotAtStartPoint. */

  PathPlannerTrajectory trajectoryPath;

  public RunTrajectorySequenceRobotAtStartPoint(String trajectory, double maxVelocity, double maxAcceleration) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand( () -> RobotContainer.driveSubsystem.resetOdometry(trajectoryPath.getInitialPose()) ),  // Set the initial pose of the robot to the one in a trajectory
      new AutonomousTrajectoryRioCommand(trajectoryPath) // Run a trajectory
    );
  }

  public RunTrajectorySequenceRobotAtStartPoint(String trajectory) {

    this(trajectory, DriveConstants.maxVelocityDefault, DriveConstants.maxAccelerationDefault);
    System.out.println("*** Run trajectory "+ trajectory);
  }

}
