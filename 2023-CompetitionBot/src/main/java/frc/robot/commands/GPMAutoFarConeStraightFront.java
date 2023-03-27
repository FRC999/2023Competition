// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.NavigationConstants;
import frc.robot.Constants.GamepieceManipulator.Arm;
import frc.robot.Constants.GamepieceManipulator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GPMAutoFarConeStraightFront extends SequentialCommandGroup {
  /** Creates a new GPMAutoFarConeStraightFront.
   * Start with front facing the target, turret rotated to the back (starting position)
   * Rotate the turret to 0 and place the element to the top row (cone or cube)
   * Retract the arm and lower the elevator
   */
  public GPMAutoFarConeStraightFront() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("Auto-cone Top row"),
      new TurretTurnToFront(),
      new PrintCommand("Turret Done"),
      new ElevatorToPredefinedHeight(Elevator.gamepieceHeights.AutoCone),
      new PrintCommand("Elevator Done"),
      new ArmToLength( () -> Arm.maximumExtension),
      new PrintCommand("Arm done"),
      new WaitCommand(0.4),
      new FlipperDown(),
      new WaitCommand(1.0),
      new ClawOpen(),
      new WaitCommand(0.5),
      new FlipperUp(),
      new GPMSemiFetalPosition()
    );
  }
}
