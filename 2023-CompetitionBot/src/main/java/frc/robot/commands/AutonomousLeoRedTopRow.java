// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousLeoRedTopRow extends SequentialCommandGroup {
  /** Creates a new AutonomousLeoRedTopRow. 
   * Faces front target
   * Deposits cube on top row
   * Runs LeoRed trajectory
   * Claw closes on cube
   * Flicks cube (Rotate turret 90, elevator up, flipper down, and open claw)
  */
  public AutonomousLeoRedTopRow() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("AutonomousLeoRedTopRow"),
      new PrintCommand("Placing Game element on the top row ..."),
      new GPMAutoFarConeStraightFront(),
      new FlipperDown(),
      //new WaitCommand(0.2),
      new PrintCommand("Moving outside the home zone..."),
      new RunTrajectorySequenceRobotAtStartPoint("LeoRed",false),
      new ClawClose(), /// grab a cube (hopefully)
      new WaitCommand(0.1),
      new FlipperUp(),
      new WaitCommand(0.1),
      new AutoFlickCubeLeftV1()
    );
  }
}
