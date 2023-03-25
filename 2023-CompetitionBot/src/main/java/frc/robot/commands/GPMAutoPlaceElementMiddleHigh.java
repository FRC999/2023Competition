// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamepieceManipulator.Arm;
import frc.robot.Constants.GamepieceManipulator.Elevator;
import frc.robot.subsystems.NavigationSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GPMAutoPlaceElementMiddleHigh extends SequentialCommandGroup {
  /** Creates a new GPMAutoPlaceElementMiddleHigh. 
   * Placing element in a middle low when the elevator is currently higher than needed
  */
  public GPMAutoPlaceElementMiddleHigh() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AcquireRobotPositionUsingLL(),

      new ConditionalCommand(
        new PrintCommand("Arm extension checked"),
        new InstantCommand(RobotContainer.elevatorSubsystem::clearMiddleCommandStarted), 
        () -> RobotContainer.navigationSubsystem.getTurretArmToTarget(0,1) < Arm.maximumExtension
      ),
      new ConditionalCommand(
        new PrintCommand(""),
        new InstantCommand(this::cancel), 
        () -> RobotContainer.navigationSubsystem.getTurretArmToTarget(0,1) < Arm.maximumExtension
      ),

      // The end() in the T E A commands was modified to clear that flag if the command is interrupted
      // The reason - if a command is interrupted, the sequence is interrupted as well
      new InstantCommand(RobotContainer.elevatorSubsystem::setMiddleCommandStarted),

      new ArmToLength( () -> RobotContainer.navigationSubsystem.getTurretArmToTarget(0,1) ),
      new TurretToAngle( () -> RobotContainer.navigationSubsystem.getTurretArmToTarget(0,0) ),
      new ElevatorToPredefinedHeight(Elevator.gamepieceHeights.MidCone),
      new WaitCommand(0.4),
      new FlipperDown(),
      new ClawOpen(),
      new WaitCommand(0.2),
      new FlipperUp()
    );
  }
}