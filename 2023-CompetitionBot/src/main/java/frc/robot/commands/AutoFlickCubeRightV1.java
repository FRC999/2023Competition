// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoFlickCubeRightV1 extends ParallelCommandGroup {
  /** Creates a new AutoFlickCubeV1. 
   * rotates 90 degrees to the right for turret to face targets from blue side
   * runs the following in parallel
   * brings up elevator
   * brings flupper down
   * opens claw
  */
  public AutoFlickCubeRightV1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new TurretToAngle(-90),
      //new AutoFlickCubeElevatorV1(),
      new AutoFlickCubeFlipperV1(),
      new AutoFlickCubeClawV1()
    );
  }
}
