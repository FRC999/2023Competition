// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.GamepieceManipulator.Turret;

public class TurretToAngle extends CommandBase {

  double angle=0;
  DoubleSupplier angleDynamic;

  private double degreeTolerance = 1.0;

  /** Creates a new TurretToAngle. */
  public TurretToAngle(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(RobotContainer.turretSubsystem);

    this.angle=angle;
  }

  public TurretToAngle(DoubleSupplier angleDynamicP) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(RobotContainer.turretSubsystem);

    this.angleDynamic=angleDynamicP;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (angleDynamic != null) {
      angle = angleDynamic.getAsDouble();
    }
    RobotContainer.turretSubsystem.moveToPosition(angle * Turret.ticksPerDegree);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Turret turned to angle "+ angle+". I:"+interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return Math.abs(RobotContainer.turretSubsystem.getDegrees() - angle) < degreeTolerance; // end command when we're close to 0
  }
}
