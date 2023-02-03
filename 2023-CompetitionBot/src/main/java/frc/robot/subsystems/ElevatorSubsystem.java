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
import frc.robot.Constants.GamepieceManipulator.Elevator;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private WPI_TalonSRX elevatorMotorController;
  public final int clicksPerFullRotationSRX = 4096;
  
  public ElevatorSubsystem() {
    initializeElevator();
    calibrateRelativeEncoder();
  }

  public void initializeElevator() {
    elevatorMotorController =  new WPI_TalonSRX(Constants.GamepieceManipulator.Elevator.elevatorMotorID);
    
    elevatorMotorController.configFactoryDefault();

    elevatorMotorController.setSafetyEnabled(false);

    // Configure motor and encoder
    elevatorMotorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
      Elevator.PID_Elevator_Idx,
      Elevator.elevator_configureTimeoutMs);
    elevatorMotorController.setSensorPhase(Elevator.elevatorSensorPhase);
    elevatorMotorController.setInverted(Elevator.elevatorMotorInverted);
    elevatorMotorController.configNeutralDeadband(Elevator.elevator_NeutralDeadband,
     Elevator.elevator_configureTimeoutMs);

    // PID configuration
    elevatorMotorController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0,
        10,
        Elevator.elevator_configureTimeoutMs);
    elevatorMotorController.setStatusFramePeriod(StatusFrame.Status_10_Targets,
        10,
        Elevator.elevator_configureTimeoutMs);

    elevatorMotorController.configPeakOutputForward(+1.0, Elevator.elevator_configureTimeoutMs);
    elevatorMotorController.configPeakOutputReverse(-1.0, Elevator.elevator_configureTimeoutMs);
    elevatorMotorController.configNominalOutputForward(0, Elevator.elevator_configureTimeoutMs);
    elevatorMotorController.configNominalOutputReverse(0, Elevator.elevator_configureTimeoutMs);

    /* FPID Gains */
    elevatorMotorController.selectProfileSlot(Elevator.elevator_SLOT_0, Elevator.PID_Elevator_Idx);
    elevatorMotorController.config_kP(Elevator.elevator_SLOT_0, Elevator.elevator_kP,
      Elevator.elevator_configureTimeoutMs);
    elevatorMotorController.config_kI(Elevator.elevator_SLOT_0, Elevator.elevator_kI,
      Elevator.elevator_configureTimeoutMs);
    elevatorMotorController.config_kD(Elevator.elevator_SLOT_0, Elevator.elevator_kD,
      Elevator.elevator_configureTimeoutMs);
    elevatorMotorController.config_kF(Elevator.elevator_SLOT_0, Elevator.elevator_kF,
      Elevator.elevator_configureTimeoutMs);

    elevatorMotorController.config_IntegralZone(Elevator.elevator_SLOT_0, Elevator.elevator_Izone,
      Elevator.elevator_configureTimeoutMs);
    elevatorMotorController.configClosedLoopPeakOutput(Elevator.elevator_SLOT_0, Elevator.elevator_PeakOutput,
      Elevator.elevator_configureTimeoutMs);
    elevatorMotorController.configAllowableClosedloopError(Elevator.elevator_SLOT_0,
      Elevator.elevator_DefaultAcceptableError,
      Elevator.elevator_configureTimeoutMs);

    elevatorMotorController.configClosedLoopPeriod(Elevator.elevator_SLOT_0, Elevator.elevator_closedLoopPeriodMs,
      Elevator.elevator_configureTimeoutMs);

    elevatorMotorController.configMotionAcceleration(Elevator.elevator_Acceleration,
      Elevator.elevator_configureTimeoutMs);
    elevatorMotorController.configMotionCruiseVelocity(Elevator.elevator_CruiseVelocity,
      Elevator.elevator_configureTimeoutMs);
    elevatorMotorController.configMotionSCurveStrength(Elevator.elevator_Smoothing);
  }

  public void zeroEncoders(){
    elevatorMotorController.setSelectedSensorPosition(0);
    System.out.println("elevator encoders zeroed");
  }

  public void stopElevator() {
    elevatorMotorController.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public int getEncoder() {
    return (int) elevatorMotorController.getSelectedSensorPosition();
  }

  public double getError() {
    return elevatorMotorController.getClosedLoopError();// Returns the PID error for Pan motion control;
  }

  public int getDriveAbsEncoder() {
    return (int) elevatorMotorController.getSensorCollection().getPulseWidthPosition() & 0xFFF;
 }

 public void calibrateRelativeEncoder() {
  double relativePosition = getDriveAbsEncoder() - Elevator.elevatorAbsoluteZero; 
  elevatorMotorController.setSelectedSensorPosition(relativePosition);
  System.out.println("Set encoder for elevator motor to " + relativePosition);
 }

 public void moveToPosition(double endingPosition) {
  elevatorMotorController.set(TalonSRXControlMode.MotionMagic,endingPosition);
  System.out.println("Elevator going to " + endingPosition);
 }
  
 public void manualDrive() {
  elevatorMotorController.set(TalonSRXControlMode.PercentOutput, Constants.GamepieceManipulator.Elevator.EdefaultPowerManual);
 }

 public void manualDrive(double power) {
  elevatorMotorController.set(TalonSRXControlMode.PercentOutput, power);
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
