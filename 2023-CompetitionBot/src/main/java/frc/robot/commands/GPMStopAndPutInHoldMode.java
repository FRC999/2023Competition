// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GPMStopAndPutInHoldMode extends SequentialCommandGroup {
  /** Creates a new GPMStopAndPutInHoldMode.
   * Stop GPM components and apply ForceFeed to Elevator and Arm
   */
  public GPMStopAndPutInHoldMode() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // Stop Turret
        new InstantCommand(RobotContainer.turretSubsystem::stopTurret, RobotContainer.turretSubsystem),
        // Stop Elevator
        new InstantCommand(RobotContainer.elevatorSubsystem::elevatorForceFeed, RobotContainer.elevatorSubsystem),
        // Stop Arm
        new InstantCommand(RobotContainer.armSubsystem::armForceFeed, RobotContainer.armSubsystem)
    );
  }
}
