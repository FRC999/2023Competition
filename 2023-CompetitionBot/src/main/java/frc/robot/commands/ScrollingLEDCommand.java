// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.CharToByteArray;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CANdleSubsystem;

/**
 * This command never ends; 
 */
public class ScrollingLEDCommand extends CommandBase {
    
  public ScrollingLEDCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.candleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    final byte[][] arraySpace={
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
        {0x00,0x00,0x00,0x00,0x00,0x00,0x00}};

    byte[][] ledMatrix;
    byte[][] ledMatrixNew = {};
    //ledMatrixNew = RobotContainer.candleSubsystem.stringToMap("hello");
    //RobotContainer.candleSubsystem.prepareToScroll(ledMatrix, -1);
   

    /*
    final ledM= byte[][] ledMatrix;
    ledMatrix = new byte[8][32];
    ledMatrixNew = RobotContainer.candleSubsystem.stringToMap("hello");
    ledMatrixNew.matrixToLed();
    RobotContainer.candleSubsystem.prepareToScroll(ledMatrix, -1);
     */

    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
