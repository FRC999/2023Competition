// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GamepieceManipulator.Turret;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */

  private WPI_TalonSRX turretMotorController;
  final int clicksPerFullRotationSRX = 4096;

  public boolean limitOverride = false;
  
  //some filler variables that will be based on limelight measurements
  //public double deltaX = 5; //this variable stores the "x" offset of the limelight relative to the turret
  //public double deltaY = 5; //this variable store the "y" offset of the limelight relative to the turret
  //public double hype = Math.sqrt(Math.pow(deltaX, 2.00) + Math.pow(deltaY, 2.00) ); //distance between center of the camera and center of the turret

  //some filler variables that will be based on limelight data
  //public double limelightFieldAngle = 0;
  //public double limelightFieldx = 15;
  //public double limelightFieldY = 15;

  //values that will be retrieved by method
  //public double turretX = 0;
  //public double turretY = 0;

  public TurretSubsystem() {
    initializeTurret();
    brakeMode();
    calibrateRelativeEncoderWhenTurnedTo180Angle();
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

    turretMotorController.configPeakOutputForward(+0.5, Turret.turret_configureTimeoutMs);
    turretMotorController.configPeakOutputReverse(-0.5, Turret.turret_configureTimeoutMs);
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

  public void brakeMode() {
    turretMotorController.setNeutralMode(NeutralMode.Brake);
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

  public int getAbsEncoder() {
    return (int) turretMotorController.getSensorCollection().getPulseWidthPosition() & 0xFFF;
 }

 public double getSpeed() {
  return turretMotorController.getSelectedSensorVelocity();
 }

 public double getDegrees() {
  return getEncoder() * Turret.turretDegreesPerTick ;
 }

 public void calibrateRelativeEncoderWhenTurnedToZeroAngle() {
  double currentAbsEncoder = getAbsEncoder();
  double relativePosition = (getAbsEncoder() < Turret.turretAbsoluteZeroClockwisePositionLimit) ?
    (Turret.turretAbsoluteZero - getAbsEncoder()) :
    (Turret.turretAbsoluteZeroRollover-getAbsEncoder()+Turret.turretAbsoluteZero); //Done because absolute encoder increases clockwise
  turretMotorController.setSelectedSensorPosition(relativePosition);
  System.out.println("*** Set relative encoder for Turret motor to " + relativePosition + " Abs:"+currentAbsEncoder);
 }

 public void calibrateRelativeEncoderWhenTurnedTo180Angle(){
  double currentAbsEncoder = getAbsEncoder();

  if(Turret.turretTurnLowerLimit < currentAbsEncoder && currentAbsEncoder < Turret.turretTurnUpperLimit){

    System.out.println("****==> Turret rotated to the right for calibration");
    turretMotorController.setSelectedSensorPosition(
      Turret.turretRightRelative180 + (Turret.turretAbsoluteZeroClockwisePositionLimitRight-currentAbsEncoder)
    );
  } else {
    System.out.println("****<== Turret rotated to the left for calibration");
    turretMotorController.setSelectedSensorPosition(
      (currentAbsEncoder<=Turret.turretAbsoluteZeroRollover) ?
        Turret.turretLeftRelative180 + (Turret.turretAbsoluteZeroClockwisePositionLimitLeft-currentAbsEncoder) :
        Turret.turretLeftRelative180 + (Turret.turretAbsoluteZeroRollover-Turret.turretAbsoluteZeroClockwisePositionLimitLeft+currentAbsEncoder)
    );
  
  }

  System.out.println("*** Set relative encoder for Turret motor to " + turretMotorController.getSelectedSensorPosition() + " Abs:"+currentAbsEncoder);

  /*double relativePosition = (getAbsEncoder() < Turret.turretAbsoluteZeroClockwisePositionLimitLeft) ?
    (Turret.turretLeftLimit - getAbsEncoder()) :
    (Turret.turretAbsoluteZeroRollover-getAbsEncoder()+Turret.turretLeftLimit); 
  turretMotorController.setSelectedSensorPosition(relativePosition);
  System.out.println("*** Set relative encoder for Turret motor to " + relativePosition + " Abs:"+getAbsEncoder());
 }

 public void calibrateRelativeEncoderOnNegativeTurn(){
  double relativePosition = (getAbsEncoder() < Turret.turretAbsoluteZeroClockwisePositionLimitRight) ?
    (Turret.turretRightLimit - getAbsEncoder()) :
    (Turret.turretAbsoluteZeroRollover-getAbsEncoder()+Turret.turretRightLimit); 
  turretMotorController.setSelectedSensorPosition(relativePosition);
  System.out.println("*** Set relative encoder for Turret motor to " + relativePosition + " Abs:"+getAbsEncoder());
  */
 }

 /**
  * Turn the turret to the relative encoder position specific
  * @param endingPosition - in ticks
  */
 public void moveToPosition(double endingPosition) {
  turretMotorController.set(TalonSRXControlMode.Position,endingPosition);
  //System.out.println("Turret PID turn to "+ endingPosition);
 }

 /**
  * Turn the turret to angle based on relative encoder; posotive angle = counterclockwise
  * @param angle - degrees
  */
 public void turnTurretToAngle(double angle) {
  System.out.println("Turn to angle "+angle);
  moveToPosition(angle * Turret.ticksPerDegree);
 }

 public void manualDrive() {
  turretMotorController.set(TalonSRXControlMode.PercentOutput, Constants.GamepieceManipulator.Turret.TdefaultPowerManual);
 }

 public void manualDrive(double power) {
  turretMotorController.set(TalonSRXControlMode.PercentOutput, power);
 }

 public void setLimitOverride() {
  limitOverride = true;
 }
 public void clearLimitOverride() {
  limitOverride = false;
 }
 public boolean getLimitOverride() {
  return limitOverride;
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
