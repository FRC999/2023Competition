// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.PneumaticsConstants;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
  /** Creates a new PneumaticsSubsystem. */
  private Compressor compressor;

  private PneumaticHub pneumaticsHub =new PneumaticHub(PneumaticsConstants.compressorCANID);

  public PneumaticsSubsystem() {
    compressor = new Compressor(PneumaticsConstants.compressorCANID, PneumaticsConstants.pneumaticsModuleType);


    System.out.println("**** Activating compressor");
    activateCompressor();

    //TODO: for testing **** !!!!!!!
    //deactivateCompressor();
  }

  public void activateCompressor() {
    compressor.enableDigital();
  }

  public void deactivateCompressor() {
    compressor.disable();
  }

  public PneumaticHub getPneumaticsHub() {
    return pneumaticsHub;
  }

  @Override
  public void periodic() {
    
  }
}