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
public class GPMAutoPlaceElementLowNoFlip extends SequentialCommandGroup {
  /** Creates a new GPMAutoPlaceElementLowNoFlip. */
  public GPMAutoPlaceElementLowNoFlip() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("Auto-place element LOW"),
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
      new ArmToLength( () -> RobotContainer.navigationSubsystem.getTurretArmToTarget(0,1) ),
      new TurretToAngle( () -> RobotContainer.navigationSubsystem.getTurretArmToTarget(0,0) )
      //new ElevatorToPredefinedHeight(Elevator.gamepieceHeights.LowCube),
    );
  }
}
