// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GamepieceManipulator.Turret;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */

  private WPI_TalonSRX turretMotorController;
  final int clicksPerFullRotationSRX = 4096;

  
  public TurretSubsystem() {
    initializeTurret();
    calibrateRelativeEncoder();
    calibrateZero();
  }

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
    turretMotorController.selectProfileSlot(Turret.turret_SLOT_0, Turret.PID_Turret_Idx);
    turretMotorController.config_kP(Turret.turret_SLOT_0, Turret.turret_kP,
      Turret.turret_configureTimeoutMs);
    turretMotorController.config_kI(Turret.turret_SLOT_0, Turret.turret_kI,
      Turret.turret_configureTimeoutMs);
    turretMotorController.config_kD(Turret.turret_SLOT_0, Turret.turret_kD,
      Turret.turret_configureTimeoutMs);
    turretMotorController.config_kF(Turret.turret_SLOT_0, Turret.turret_kF,
      Turret.turret_configureTimeoutMs);

    turretMotorController.config_IntegralZone(Turret.turret_SLOT_0, Turret.turret_Izone,
      Turret.turret_configureTimeoutMs);
    turretMotorController.configClosedLoopPeakOutput(Turret.turret_SLOT_0, Turret.turret_PeakOutput,
      Turret.turret_configureTimeoutMs);
    turretMotorController.configAllowableClosedloopError(Turret.turret_SLOT_0,
      Turret.turret_DefaultAcceptableError,
      Turret.turret_configureTimeoutMs);

    turretMotorController.configClosedLoopPeriod(Turret.turret_SLOT_0, Turret.turret_closedLoopPeriodMs,
      Turret.turret_configureTimeoutMs);

    turretMotorController.configMotionAcceleration(Turret.turret_Acceleration,
      Turret.turret_configureTimeoutMs);
    turretMotorController.configMotionCruiseVelocity(Turret.turret_CruiseVelocity,
      Turret.turret_configureTimeoutMs);
    turretMotorController.configMotionSCurveStrength(Turret.turret_Smoothing);
  }

  public void zeroEncoders(){
    turretMotorController.setSelectedSensorPosition(0);
    System.out.println("turret encoders zeroed");
  }

  public void stopTurret() {
    turretMotorController.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public int getEncoder() {
    return (int) turretMotorController.getSelectedSensorPosition();
  }

  public double getError() {
    return turretMotorController.getClosedLoopError();// Returns the PID error for Pan motion control;
  }

  public void calibrateZero() {
    zeroEncoders();
    calibrateRelativeEncoder();
  }

  public int getDriveAbsEncoder() {
    return (int) turretMotorController.getSensorCollection().getPulseWidthPosition() & 0xFFF;
 }

 public void calibrateRelativeEncoder() {
  double relativePosition = getDriveAbsEncoder() - Turret.turretAbsoluteZero; 
  turretMotorController.setSelectedSensorPosition(relativePosition);
  System.out.println("Set encoder for elevator motor to " + relativePosition);
 }

 public void moveToPosition(double endingPosition) {
  turretMotorController.set(TalonSRXControlMode.MotionMagic,endingPosition);
  System.out.println(endingPosition);
 }

 public void manualDrive() {
  turretMotorController.set(TalonSRXControlMode.PercentOutput, Constants.GamepieceManipulator.Turret.TdefaultPowerManual);
 }

 public void manualDrive(double power) {
  turretMotorController.set(TalonSRXControlMode.PercentOutput, power);
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
