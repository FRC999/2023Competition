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

  private final int LEDOFFSET = 8; // move LED numbers because there are 8 LEDs on the controller

  private CANdle candle;

  private byte[][] ledMatrix;

  private int currentMatrixColumn; // for the scrolling matrix - location of the first/last LED column of the image matrix in the LED matrix
  private int scrollDirection = -1; // Negative - scroll left; otherwise scroll right
  private byte[][] scrollImage; // placeholder for the scroll image
  private byte[][] origImage; // keep a copy of the original image - just in case

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
    ledMatrix = new byte[8][32];

    for(int i = 0; i<ledS.length(); i++){
      for(int j=0; j<7; j++){
        for(int k=0;k<7;k++) {
          ledMatrix[j][i*8+k] = CharToByteArray.decodeMap.get(ledS.charAt(i))[j][k];
        }
      }
    }
  }

  public void matrixToLed() {
    for (int i=0; i<32; i++) {
      for (int j=0;j<8;j++) {
        // Every even column is right direction
        // Every odd column - in reverse

        System.out.println("I:"+i+" J:"+j);
        if(i%2==0) {
          if (ledMatrix[j][i] == 0x01) {
            setOneLEDToColor(new int[]{200,10,10},i*8+j+LEDOFFSET);
          } else {
            setOneLEDToColor(new int[]{0,0,0},i*8+j+LEDOFFSET);
          }
        } else {
          if (ledMatrix[j][i] == 0x01) {
            setOneLEDToColor(new int[]{200,10,10},i*8+(7-j)+LEDOFFSET);
          } else {
            setOneLEDToColor(new int[]{0,0,0},i*8+j+LEDOFFSET);
          }
        }
      }
    }
  }

  public void printMsg(String s) {
    stringToMap(s);
    matrixToLed();
  }

  /**
   * Initialize scrolling
   * @param image - byte[][] array that contains the image to scroll; must have the same number of rows as LED matrix
   * @param direction - negative - scroll left; otherwise - scroll right
   */
  public void prepareToScroll(byte[][] image, int direction) {
    // Image for scrolling must be the same or bigger than the LED matrix and have the same number of rows
    if (image.length != CANdleConstants.ledMatrixRows || image[0].length < CANdleConstants.ledMatrixColumns) {
      System.out.println("==== Image is unsuitable for scrolling");
      return;
    }
    setLEDOff(); // turn off all LEDs
    ledMatrix = new byte[CANdleConstants.ledMatrixRows][CANdleConstants.ledMatrixColumns];
    scrollImage = image;
    scrollDirection = direction;
    origImage = scrollImage.clone(); // save a copy of the original image - just in case; came remove if the RAM becomes an issue
  }

  /**
   * Display current image based on the currentMatrixColumn
   * Move scrolling image to the next line indicated by the currentMatrixColumn
   */
  public void moveScroll(){

    // Current image to LED matrix
    for (int i=0;i<CANdleConstants.ledMatrixRows;i++) {
      for (int j=0;i<CANdleConstants.ledMatrixColumns;i++) {
        ledMatrix[i][j] = scrollImage[i][j] ;
      }
    }
    // Display LED matrix
    matrixToLed();

    // Shift the image according to the direction
    byte[] tmpCol= new byte[scrollImage.length]; // temp place to hold the column that needs to go to the other side

    if (scrollDirection<0) { // Scroll to the left
      for(int i=0;i<CANdleConstants.ledMatrixRows;i++) {
        tmpCol[i]=scrollImage[i][0]; // Save the first column
      }
      for (int i=0;i<CANdleConstants.ledMatrixRows;i++) {
        for (int j=0;j<scrollImage[0].length-2;j++) {
          scrollImage[i][j] = scrollImage[i][j+1] ; // Shift columns to the left
        }
      }
      for (int i=0;i<CANdleConstants.ledMatrixRows;i++) {
        scrollImage[i][scrollImage[0].length-1]=tmpCol[i]; // Put saved values in the last column
      }
    } else { // Scroll to the right
      for(int i=0;i<CANdleConstants.ledMatrixRows;i++) {
        tmpCol[i]=scrollImage[i][scrollImage[0].length-1]; // Save the last column
      }
      for (int i=0;i<CANdleConstants.ledMatrixRows;i++) {
        for (int j=scrollImage[0].length-1;j>0;j--) {
          scrollImage[i][j] = scrollImage[i][j-1] ; // Shift columns to the right
        }
      }
      for (int i=0;i<CANdleConstants.ledMatrixRows;i++) {
        scrollImage[i][0]=tmpCol[i]; // Put saved values in the first column
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
