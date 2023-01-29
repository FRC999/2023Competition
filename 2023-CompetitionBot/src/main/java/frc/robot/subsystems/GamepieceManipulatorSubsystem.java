// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GamepieceManipulator;
import frc.robot.Constants.GamepieceManipulator.Turret;

public class GamepieceManipulatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  private WPI_TalonSRX turretMotorController;
  private WPI_TalonSRX elevatorMotorController;
  private WPI_TalonSRX armMotorController;
  
  public GamepieceManipulatorSubsystem() {
    initializeTurret();
    initializeElevator();
    initializeArm();
  }

  /**
   * Initialize turret motor and set for MotionMagic
   */
  private void initializeTurret() {
    turretMotorController =  new WPI_TalonSRX(Turret.turretMotorID);

    turretMotorController.configFactoryDefault();

    turretMotorController.setSafetyEnabled(false);

    // Configure motor and encoder
    turretMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
      Turret.PID_Turret_Idx,
      Turret.turret_configureTimeoutMs);
    turretMotorController.setSensorPhase(Turret.turretSensorPhase);
    turretMotorController.setInverted(Turret.turretMotorInverted);
    turretMotorController.configNeutralDeadband(Turret.turret_NeutralDeadband,
     Turret.turret_configureTimeoutMs);

    // PID configuration
    turretMotorController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,
        10,
        Turret.turret_configureTimeoutMs);
    turretMotorController.setStatusFramePeriod(StatusFrame.Status_10_Targets,
        10,
        Turret.turret_configureTimeoutMs);

    turretMotorController.configPeakOutputForward(+1.0, Turret.turret_configureTimeoutMs);
    turretMotorController.configPeakOutputReverse(-1.0, Turret.turret_configureTimeoutMs);
    turretMotorController.configNominalOutputForward(0, Turret.turret_configureTimeoutMs);
    turretMotorController.configNominalOutputReverse(0, Turret.turret_configureTimeoutMs);

    /* FPID Gains */
    turretMotorController.selectProfileSlot(0, 0);
    turretMotorController.config_kP(0, 0.75, 30);
    turretMotorController.config_kI(0, 0.005, 30);
    turretMotorController.config_kD(0, 0.01, 30);
    turretMotorController.config_kF(0, 0, 30);

  }

  /**
   * Initialize elevator motor and set for MotionMagic
   */
  private void initializeElevator() {
    elevatorMotorController =  new WPI_TalonSRX(Constants.GamepieceManipulator.Elevator.elevatorMotorID);

    
  }

    /**
   * Initialize arm motor and set for MotionMagic
   */
  private void initializeArm() {
    turretMotorController =  new WPI_TalonSRX(Constants.GamepieceManipulator.Arm.armMotorID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
