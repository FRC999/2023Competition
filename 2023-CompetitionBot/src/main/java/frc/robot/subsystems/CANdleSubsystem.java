// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANdleConstants;

import frc.robot.CharToByteArray;

public class CANdleSubsystem extends SubsystemBase {

  private CANdle candle;

  private byte[][] ledMatrix;

  /** Creates a new CANdleSubsystem. */
  public CANdleSubsystem() {
    System.out.println("Initializing CANdle");

    candle = new CANdle(CANdleConstants.CANdlePort);

    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = false;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(configAll, 100);

    // m_candle.setLEDs(50, 60, 70, 80, 0, 2);
    System.out.println("CANdle initalization complete");
    
  }

  public void setAllLEDToColor(int[] rgb) {
    candle.setLEDs(rgb[0], rgb[1], rgb[2]);
    candle.modulateVBatOutput(0.9);
  }

  // Turn one LED to a specific color
  public void setOneLEDToColor(int[] rgb, int ledNumber) {
    candle.setLEDs(rgb[0], rgb[1], rgb[2], 0, ledNumber, 1);
    candle.modulateVBatOutput(0.9);
  }

  public void setLEDOff() {
    candle.setLEDs(0, 0, 0);
    candle.modulateVBatOutput(0);
  }

  public void stringToMap(String s) {
    String ledS = (s.length()>4)?s.substring(0, 4):s;
    ledMatrix = new byte[8][ledS.length()*8];

    for(int i = 0; i<ledS.length(); i++){
      for(int j = 0; j<7; j++){
        ledMatrix[i*8][i*8+j] = CharToByteArray.decodeMap.get(ledS.substring(i, i+1))[i*8][j];
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
