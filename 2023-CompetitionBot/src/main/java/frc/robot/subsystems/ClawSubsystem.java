// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.GamepieceManipulator.Claw;

public class ClawSubsystem extends SubsystemBase {
  private static DoubleSolenoid clawSolenoid;
  private static DoubleSolenoid flipperSolenoid;
  private WPI_TalonSRX clawMotorController;

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    // clawSolenoid = new DoubleSolenoid( PneumaticsConstants.pneumaticsModuleType,
    // Claw.clawSolenoidChannels[0], Claw.clawSolenoidChannels[1]);
    // flipperSolenoid = new DoubleSolenoid(
    // PneumaticsConstants.pneumaticsModuleType, Claw.flipperSolenoidChannels[0],
    // Claw.flipperSolenoidChannels[1]);
    clawSolenoid = RobotContainer.pneumaticsSubsystem.getPneumaticsHub()
        .makeDoubleSolenoid(Claw.clawSolenoidChannels[0], Claw.clawSolenoidChannels[1]);
    flipperSolenoid = RobotContainer.pneumaticsSubsystem.getPneumaticsHub()
        .makeDoubleSolenoid(Claw.flipperSolenoidChannels[0], Claw.flipperSolenoidChannels[1]);

    initializeClaw();
    brakeMode();
    flipperUp();
    stopClaw();

    
  }

  private void initializeClaw() {
    clawMotorController = new WPI_TalonSRX(Constants.GamepieceManipulator.Claw.clawMotorID);

    clawMotorController.configFactoryDefault();

    clawMotorController.setSafetyEnabled(false);

  }

  public void brakeMode() {
    clawMotorController.setNeutralMode(NeutralMode.Brake);
  }

  public void stopClaw() {
    clawMotorController.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void closeClaw() {
    //clawSolenoid.set(Value.kForward);
    clawMotorController.set(TalonSRXControlMode.PercentOutput, -0.5);
  }

  public void openClaw() {
    //clawSolenoid.set(Value.kReverse);
    clawMotorController.set(TalonSRXControlMode.PercentOutput, 0.5);
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
