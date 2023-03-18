// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ArmToLength extends CommandBase {

  private double armLength; // extended arm length - from the center of the turret
  DoubleSupplier armLengthDynamic;
  private double lengthTolerance = 0.02; // meters 
  DoubleSupplier dynamicArmLength;


  /** Extend arm to specific length (meters) */
  public ArmToLength(double aLength) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.armSubsystem);
    armLength = aLength;
  }

  public ArmToLength(DoubleSupplier dynamicALength){
    addRequirements(RobotContainer.armSubsystem);
    this.dynamicArmLength = dynamicALength;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (dynamicArmLength != null) {
      armLength = dynamicArmLength.getAsDouble();
    }
    RobotContainer.armSubsystem.extendArmToLengthMeters(armLength);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Arm extended to "+armLength);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(armLength - RobotContainer.armSubsystem.getLength()) <= lengthTolerance;
  }
}
