// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GamepieceManipulator.Arm;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private WPI_TalonSRX armMotorController;
  public static final int clicksPerFullRotationSRX = 4096;
  
  public ArmSubsystem() {
    initializeArm();
    brakeMode();
    calibrateRelativeEncoder();
  }

  private void initializeArm() {
    armMotorController =  new WPI_TalonSRX(Constants.GamepieceManipulator.Arm.armMotorID);

    armMotorController.configFactoryDefault();

    armMotorController.setSafetyEnabled(false);

    // Configure motor and encoder
    armMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
      Arm.PID_Arm_Idx,
      Arm.arm_configureTimeoutMs);
    armMotorController.setSensorPhase(Arm.armSensorPhase);
    armMotorController.setInverted(Arm.armMotorInverted);
    armMotorController.configNeutralDeadband(Arm.arm_NeutralDeadband,
    Arm.arm_configureTimeoutMs);

    // PID configuration
    armMotorController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,
        10,
        Arm.arm_configureTimeoutMs);
    armMotorController.setStatusFramePeriod(StatusFrame.Status_10_Targets,
        10,
        Arm.arm_configureTimeoutMs);

    armMotorController.configPeakOutputForward(+1.0, Arm.arm_configureTimeoutMs);
    armMotorController.configPeakOutputReverse(-1.0, Arm.arm_configureTimeoutMs);
    armMotorController.configNominalOutputForward(0, Arm.arm_configureTimeoutMs);
    armMotorController.configNominalOutputReverse(0, Arm.arm_configureTimeoutMs);

    /* FPID Gains */
    armMotorController.selectProfileSlot(Arm.arm_SLOT_0, Arm.PID_Arm_Idx);
    armMotorController.config_kP(Arm.arm_SLOT_0, Arm.arm_kP,
    Arm.arm_configureTimeoutMs);
    armMotorController.config_kI(Arm.arm_SLOT_0, Arm.arm_kI,
    Arm.arm_configureTimeoutMs);
    armMotorController.config_kD(Arm.arm_SLOT_0, Arm.arm_kD,
    Arm.arm_configureTimeoutMs);
    armMotorController.config_kF(Arm.arm_SLOT_0, Arm.arm_kF,
    Arm.arm_configureTimeoutMs);

    armMotorController.config_IntegralZone(Arm.arm_SLOT_0, Arm.arm_Izone,
    Arm.arm_configureTimeoutMs);
    armMotorController.configClosedLoopPeakOutput(Arm.arm_SLOT_0, Arm.arm_PeakOutput,
    Arm.arm_configureTimeoutMs);
    armMotorController.configAllowableClosedloopError(Arm.arm_SLOT_0,
    Arm.arm_DefaultAcceptableError,
    Arm.arm_configureTimeoutMs);

    armMotorController.configClosedLoopPeriod(Arm.arm_SLOT_0, Arm.arm_closedLoopPeriodMs,
    Arm.arm_configureTimeoutMs);

    armMotorController.configMotionAcceleration(Arm.arm_Acceleration,
    Arm.arm_configureTimeoutMs);
    armMotorController.configMotionCruiseVelocity(Arm.arm_CruiseVelocity,
    Arm.arm_configureTimeoutMs);
    armMotorController.configMotionSCurveStrength(Arm.arm_Smoothing);
  }

  public void zeroEncoder(){
    armMotorController.setSelectedSensorPosition(0);
    System.out.println("arm encoders zeroed");
  }

  public void stopArm() {
    armMotorController.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public int getEncoder() {
    return (int) armMotorController.getSelectedSensorPosition();
  }

  public double getError() {
    return armMotorController.getClosedLoopError(); // Returns the PID error for Pan motion control;
  }

  public int getAbsEncoder() {
    return (int) armMotorController.getSensorCollection().getPulseWidthPosition() & 0xFFF;
  }

  public double getSpeed() {
    return armMotorController.getSelectedSensorVelocity();
  }

   /**
  * Get the height of the Arm from the ground
  * @return - meters
  */
 public double getLength() {
  return getEncoder() * Arm.armMetersPerTick + Arm.armLengthWhenFullyFolded;
 }


 public void calibrateRelativeEncoder() {
  double relativePosition = getAbsEncoder() - Arm.armAbsoluteZero;
  relativePosition = (Arm.armMotorInverted^Arm.armSensorPhase)?-relativePosition:relativePosition; 
  armMotorController.setSelectedSensorPosition(relativePosition);
  System.out.println("*** Set relative encoder for Arm motor to " + relativePosition);
 }

 public void moveToPosition(double endingPosition) {
  armMotorController.set(TalonSRXControlMode.MotionMagic,endingPosition);
  System.out.println("Extending to "+endingPosition);
 }

 /**
  * Extend arm to specific length from the center of the turret (meters)
  * will account for the arm length when fully folded
  * @param toLength
  */
 public void extendArmToLengthMeters(double toLength) {
  System.out.println("Extend to length "+toLength);
  moveToPosition( (toLength - Arm.armLengthWhenFullyFolded) * Arm.armTicksPerMeter);
 }

 public void manualDrive() {
  armMotorController.set(TalonSRXControlMode.PercentOutput, Constants.GamepieceManipulator.Arm.AdefaultPowerManual);
 }

 public void manualDrive(double power) {
  armMotorController.set(TalonSRXControlMode.PercentOutput, power);
 }

 public void brakeMode() {
  armMotorController.setNeutralMode(NeutralMode.Brake);
 }

 public void coastMode() {
  armMotorController.setNeutralMode(NeutralMode.Coast);
 }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
