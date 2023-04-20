// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.GamepieceManipulator.Claw;

public class ClawSubsystem extends SubsystemBase {
  private static DoubleSolenoid clawSolenoid;
  private static DoubleSolenoid flipperSolenoid;

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    clawSolenoid = new DoubleSolenoid( PneumaticsConstants.pneumaticsModuleType, Claw.clawSolenoidChannels[0], Claw.clawSolenoidChannels[1]);
    flipperSolenoid = new DoubleSolenoid( PneumaticsConstants.pneumaticsModuleType, Claw.flipperSolenoidChannels[0], Claw.flipperSolenoidChannels[1]);
    //clawSolenoid = RobotContainer.pneumaticsSubsystem.getPneumaticsHub().makeDoubleSolenoid(Claw.clawSolenoidChannels[0], Claw.clawSolenoidChannels[1]);
    //flipperSolenoid = RobotContainer.pneumaticsSubsystem.getPneumaticsHub().makeDoubleSolenoid(Claw.flipperSolenoidChannels[0], Claw.flipperSolenoidChannels[1]);

    flipperUp();
    closeClaw();
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
