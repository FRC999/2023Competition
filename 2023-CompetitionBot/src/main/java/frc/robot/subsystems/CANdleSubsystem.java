// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANdleConstants;

public class CANdleSubsystem extends SubsystemBase {

  private CANdle candle;

  /** Creates a new CANdleSubsystem. */
  public CANdleSubsystem() {

    System.out.println("Initializing CANdle");

    candle = new CANdle(CANdleConstants.CANdlePort);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
