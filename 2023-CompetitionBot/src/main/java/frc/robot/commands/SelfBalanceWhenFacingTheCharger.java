// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants.SelfBalance;
import edu.wpi.first.math.MathUtil;

public class SelfBalanceWhenFacingTheCharger extends CommandBase {
  /** Creates a new DriveForwardUntilPitch. */

  double climbingPower;
  double targetPitch;
  boolean initialPitchLowerThanTarget;
  boolean climbForward;

  boolean continueBalance = true; // True - do not stop afgter the first phase - when detected beiong on the ramp
  boolean rampReached = false;  // Flag that indicates whether we detected that we are on the ramp
  double balancePitch = 0;
  //double poorMaxClimbingPower = 0.09; //was 0.09
  double poorMaxClimbingPitch = 15; // Pitch from which we start to reduce the motor power
  double angleTolerance = 14.0; // Angle when we try to stop the robot in Phase-2

  /**
   * Drive forward with "power" until "pitch" is detected . It's assumed that you start with 
   * @param climbingPowerPhase1 - 0..+1; should be positive number; the sign is determined by climbForward parameter
   * @param targetPitch   final pitch to balance on -90..90
   * @param initialPitchLowerThanTarget  true - initial pitch assumed to be lower than target; false - initial pitch assumed to be higher than target
   * @param climbForward  true - initial climb with the front of the robot facing the target; false - initial climb with the back of the robot facing the target
   * For backwards climb need to reverse the sign of the poorMaxClimbingPitch
   */
  public SelfBalanceWhenFacingTheCharger (double climbingPowerPhase1, double targetPitch, boolean initialPitchLowerThanTarget, boolean climbForward) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveSubsystem, RobotContainer.pigeonIMUSubsystem);
    this.climbingPower=Math.abs(climbingPowerPhase1)*((climbForward)?1:-1); // Make sure the power is positive; we reverse it if needed based on climbForward
    this.targetPitch=targetPitch;
    this.initialPitchLowerThanTarget=initialPitchLowerThanTarget;
    this.climbForward=climbForward;
    
    // When initially climbing backwards, need to stop Phase-1 fast climb at negative angle
    // Phase-2 can do slow climb from any angle
    poorMaxClimbingPitch = (climbForward)?poorMaxClimbingPitch:-poorMaxClimbingPitch;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rampReached = false;
    RobotContainer.driveSubsystem.driveForward(climbingPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rampReached && continueBalance) {
      double multiplier; // Neet to reduce power when geting close to the angle in a PID-like fashion
      multiplier = (RobotContainer.pigeonIMUSubsystem.getPitch() - balancePitch) / poorMaxClimbingPitch;
      climbingPower = SelfBalance.poorMaxClimbingPower * multiplier;

      // Cap positive and negative power to the -poorMaxClimbingPower..poorMaxClimbingPower range
      climbingPower = MathUtil.clamp(climbingPower,-SelfBalance.poorMaxClimbingPower,SelfBalance.poorMaxClimbingPower);

      // Cut the power to 0 once my angle is below angleTolerance for the targetPitch
      climbingPower = (Math.abs(RobotContainer.pigeonIMUSubsystem.getPitch()) - targetPitch < angleTolerance) ? 0 : climbingPower;

      RobotContainer.driveSubsystem.driveForward(climbingPower);

      System.out.println("P:" + climbingPower + " A:" + RobotContainer.pigeonIMUSubsystem.getPitch());
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
      return rampReached && ! continueBalance; // if we reached the target pitch in Phase-1 and do not need to continue balance in Phase-2, end
    } else {
      return false;
    }
  }
}
