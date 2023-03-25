// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.DriveConstants.BigFoot;

public class BigFootSubsystem extends SubsystemBase {
  /** Creates a new BigFootSubsystem. */

  private static DoubleSolenoid footSolenoid;

  public BigFootSubsystem() {
    //footSolenoid = new DoubleSolenoid( PneumaticsConstants.pneumaticsModuleType, BigFoot.footSolenoidChannels[0], BigFoot.footSolenoidChannels[1]);
    footSolenoid = RobotContainer.pneumaticsSubsystem.getPneumaticsHub().makeDoubleSolenoid(BigFoot.footSolenoidChannels[0], BigFoot.footSolenoidChannels[1]);

    footUp();
  }

  public void footDown() {
    footSolenoid.set(Value.kForward);
  }

  public void footUp() {
    footSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
