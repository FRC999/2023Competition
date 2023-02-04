// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  private static DoubleSolenoid solenoid;

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    solenoid = new DoubleSolenoid( PneumaticsModuleType.CTREPCM, Constants.PneumaticsConstants.SolenoidGripperClawChannel[0], Constants.PneumaticsConstants.SolenoidGripperClawChannel[1]);
  }

  public void extendCylinder() {
    solenoid.set(Value.kForward);
  }

  public void retractCylinder() {
    solenoid.set(Value.kReverse);
  }

  public void toggleCylinder() {
    solenoid.toggle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
