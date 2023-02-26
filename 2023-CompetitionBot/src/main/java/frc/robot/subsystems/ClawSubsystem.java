// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GamepieceManipulator.Claw;

public class ClawSubsystem extends SubsystemBase {
  private static DoubleSolenoid clawSolenoid;
  private static DoubleSolenoid flipperSolenoid;

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    clawSolenoid = new DoubleSolenoid( PneumaticsModuleType.CTREPCM, Claw.clawSolenoidChannels[0], Claw.clawSolenoidChannels[1]);
    flipperSolenoid = new DoubleSolenoid( PneumaticsModuleType.CTREPCM, Claw.flipperSolenoidChannels[0], Claw.flipperSolenoidChannels[1]);
    flipperUp();
    openClaw();
  }

  public void closeClaw() {
    clawSolenoid.set(Value.kForward);
  }

  public void openClaw() {
    clawSolenoid.set(Value.kReverse);
  }

  public void flipperUp() {
    flipperSolenoid.set(Value.kForward);
  }

  public void flipperDown() {
    flipperSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
