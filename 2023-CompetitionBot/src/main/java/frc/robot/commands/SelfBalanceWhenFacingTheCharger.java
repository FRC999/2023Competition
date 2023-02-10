// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.SelfBalance;

public class SelfBalanceWhenFacingTheCharger extends CommandBase {
  /** Creates a new DriveForwardUntilPitch. */

  double power;
  double targetPitch;
  boolean initialPitchLowerThanTarget;

  boolean continueBalance = true; // True - do not stop afgter the first phase - when detected beiong on the ramp
  boolean rampReached = false;  // Flag that indicates whether we detected that we are on the ramp
  double balancePitch = 0;
  //double poorMaxClimbingPower = 0.09; //was 0.09
  double poorMaxClimbingPitch = 15; // Pitch from which we start to reduce the motor power
  double angleTolerance = 14.0;

  /**
   * Drive forward with "power" until "pitch" is detected . It's assumed that you start with 
   * @param power - -1..+1; positive number - forward
   * @param targetPitch   -90..90; positive pitch means front faces UP
   * @param initialPitchLowerThanTarget  true - initial pitch assumed to be lower than target; false - initial pitch assumed to be higher than target
   */
  public SelfBalanceWhenFacingTheCharger (double power, double targetPitch, boolean initialPitchLowerThanTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem, RobotContainer.pigeonIMUSubsystem);
    this.power=power;
    this.targetPitch=targetPitch;
    this.initialPitchLowerThanTarget=initialPitchLowerThanTarget;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rampReached = false;
    RobotContainer.driveSubsystem.driveForward(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rampReached && continueBalance) {
      double multiplier; // Neet to reduce power when geting close to the angle in a PID-like fashion
      multiplier = (RobotContainer.pigeonIMUSubsystem.getPitch() - balancePitch) / poorMaxClimbingPitch;
      power = SelfBalance.poorMaxClimbingPower * multiplier;

      // Cap positive and negative power to the -poorMaxClimbingPower..poorMaxClimbingPower range
      power = (power < -SelfBalance.poorMaxClimbingPower) ? -SelfBalance.poorMaxClimbingPower : power;
      power = (power > SelfBalance.poorMaxClimbingPower) ? SelfBalance.poorMaxClimbingPower : power;

      // Cut the power to 0 once my angle is below angleTolerance for the targetPitch
      power = (Math.abs(RobotContainer.pigeonIMUSubsystem.getPitch()) - targetPitch < angleTolerance) ? 0 : power;

      RobotContainer.driveSubsystem.driveForward(power);

      System.out.println("P:" + power + " A:" + RobotContainer.pigeonIMUSubsystem.getPitch());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.driveSubsystem.stopRobot();
    System.out.println("Reached target pitch "+targetPitch+" I:"+interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (! rampReached) {
     rampReached = (initialPitchLowerThanTarget)?
      (targetPitch<=RobotContainer.pigeonIMUSubsystem.getPitch()):
      (targetPitch>=RobotContainer.pigeonIMUSubsystem.getPitch())
      ;
      return rampReached && ! continueBalance; // if we reached the target pitch and do not need to continue balance, end
    } else {
      return false;
    }
  }
}
