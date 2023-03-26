// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.NavigationConstants;
import frc.robot.Constants.GamepieceManipulator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GPMAutoMidConeStraight extends SequentialCommandGroup {
  /** Creates a new GPMAutoMidConeStraight. */
  public GPMAutoMidConeStraight() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TurretTurnToFront(),
      new ElevatorToPredefinedHeight(Elevator.gamepieceHeights.HighCone),
      new ArmToLength( () -> NavigationConstants.autoMidConeLength),
      new WaitCommand(0.4),
      new FlipperDown(),
      new WaitCommand(1.0),
      new ClawOpen(),
      new WaitCommand(0.5),
      new FlipperUp()
    );
  }
}
