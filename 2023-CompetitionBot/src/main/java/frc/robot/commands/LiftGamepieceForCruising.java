// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GamepieceManipulator.Arm;
import frc.robot.Constants.GamepieceManipulator.Elevator;
import frc.robot.Constants.GamepieceManipulator.Turret;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LiftGamepieceForCruising extends SequentialCommandGroup {
  /** Creates a new LiftGamepieceForCruising. */
  public LiftGamepieceForCruising() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.parallel(
        new InstantCommand(RobotContainer.clawSubsystem::flipperUp,RobotContainer.clawSubsystem),  // Raise flipper UP
        new MoveElevatorToPredefinedHeight(Elevator.gamepieceHeights.Cruising),  // Raise the elevator to the cruising height
        new InstantCommand(() -> RobotContainer.armSubsystem.moveToPosition(Arm.armCruisingPosition), RobotContainer.armSubsystem ) // Retract Arm to a cruising position
      )
      .andThen(new InstantCommand(() -> RobotContainer.turretSubsystem.moveToPosition(Turret.turretCruisingPosition)
        ,RobotContainer.turretSubsystem)) // Move turret to a cruising position
    );
  }
}
