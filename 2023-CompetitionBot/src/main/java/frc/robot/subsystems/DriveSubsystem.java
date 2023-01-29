// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotDriveChassisConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  private WPI_TalonFX[] rightDriveTalonFX = new WPI_TalonFX[DriveConstants.rightMotorPortID.length];
  private WPI_TalonFX[] leftDriveTalonFX = new WPI_TalonFX[DriveConstants.leftMotorPortID.length];
  private DifferentialDrive drive;

  // Software Trajectory navigation
  private DifferentialDriveOdometry odometry;
  
  public DriveSubsystem() {

    /**
     * create objects for the left-side and right-side motors reset controllers to
     * defaults setup followers set controller orientation set encoder phase
     */

    System.out.println("Primary Motor - Left " + DriveConstants.leftMotorPortID[0] + " Primary Motor - Right "
        + DriveConstants.rightMotorPortID[0] + " Number of motors per side " + DriveConstants.rightMotorPortID.length);

    // Setting up motors on the left side
    for (int motor = 0; motor < DriveConstants.leftMotorPortID.length; motor++) {
      leftDriveTalonFX[motor] = new WPI_TalonFX(DriveConstants.leftMotorPortID[motor]);
      leftDriveTalonFX[motor].configFactoryDefault(); // reset the motor controller to defaults
      if (motor == 0) { // setup master
        leftDriveTalonFX[motor].set(ControlMode.PercentOutput, 0); // set the motor to Percent Output with Default of 0
        leftDriveTalonFX[motor].setInverted(DriveConstants.MotorInvert[0]);
      } else { // setup followers
        leftDriveTalonFX[motor].follow(leftDriveTalonFX[0]);
        leftDriveTalonFX[motor].setInverted(InvertType.FollowMaster); // set green lights when going forward
        System.out.println("Left Follower " + motor);
      }

      leftDriveTalonFX[motor].setSafetyEnabled(false);  // Disable motor safety - used for Motion Magic and trajectories
    }

    // Setting up motors on the right side
    for (int motor = 0; motor < DriveConstants.rightMotorPortID.length; motor++) {
      rightDriveTalonFX[motor] = new WPI_TalonFX(DriveConstants.rightMotorPortID[motor]);
      rightDriveTalonFX[motor].configFactoryDefault(); // reset the controller to defaults

      if (motor == 0) { // setup master
        rightDriveTalonFX[motor].set(ControlMode.PercentOutput, 0); // set the motor to Percent Output with Default of 0
        rightDriveTalonFX[motor].setInverted( DriveConstants.MotorInvert[1]);
      } else { // setup followers
        rightDriveTalonFX[motor].follow(rightDriveTalonFX[0]);
        rightDriveTalonFX[motor].setInverted(InvertType.FollowMaster); // set green lights when going forward
        System.out.println("Right Follower " + motor);
      }

      rightDriveTalonFX[motor].setSafetyEnabled(false);
    }

    // Engage brake mode
    driveTrainBrakeMode();

    configureEncoders();

    configureSimpleMagic();

    zeroDriveEncoders();

    drive = new DifferentialDrive(leftDriveTalonFX[0], rightDriveTalonFX[0]);
    drive.setSafetyEnabled(false); // safety must be disabled siince we plan to use Motion Magic

    // Set initial odometry for trajectories - where robot is right now from the encoder and IMU point of view
    odometry =
        new DifferentialDriveOdometry(
          RobotContainer.pigeonIMUSubsystem.getRotation2d(),
          TranslateDistanceIntoMeters(getLeftEncoder()),
          TranslateDistanceIntoMeters(-getRightEncoder())
        );

  }
    
  public void feed() {
    drive.feed();
  }

  /**
   * Talon-based PID configuration goes here
   */
  public void configureSimpleMagic() {
  }

  /**
   * Engage Brake Mode
   */
  public void driveTrainBrakeMode() {
    for (int motor = 0; motor < DriveConstants.rightMotorPortID.length; motor++) {
      rightDriveTalonFX[motor].setNeutralMode(NeutralMode.Brake);
    }
    for (int motor = 0; motor < DriveConstants.leftMotorPortID.length; motor++) {
      leftDriveTalonFX[motor].setNeutralMode(NeutralMode.Brake);
    }
  }

  public void driveTrainCoastMode() {
    for (int motor = 0; motor < DriveConstants.rightMotorPortID.length; motor++) {
      rightDriveTalonFX[motor].setNeutralMode(NeutralMode.Coast);
    }
    for (int motor = 0; motor < DriveConstants.leftMotorPortID.length; motor++) {
      leftDriveTalonFX[motor].setNeutralMode(NeutralMode.Coast);
    }
  }

  public double getEncoderTicksPerInch() {
    // tics per rotation / number of inches per rotation * gearReduction
    return RobotDriveChassisConstants.encoderUnitsPerMotorRotation
        / (RobotDriveChassisConstants.wheelDiameter * Math.PI) * RobotDriveChassisConstants.encoderGearReduction;
  }

  public void setLeftVoltage(double voltage) {
    rightDriveTalonFX[0].setVoltage(voltage);
  }

  public void setRightVoltage(double voltage) {
    rightDriveTalonFX[0].setVoltage(voltage);
  }

  public double deadbandMove(double move) {
    if (Math.abs(move) >= DriveConstants.deadbandY) {
      if (move > 0) {
        move = (move - DriveConstants.deadbandY) / (1 - DriveConstants.deadbandY);
      } else {
        move = (move + DriveConstants.deadbandY) / (1 - DriveConstants.deadbandY);
      }
    } else {
      move = 0;
    }
    return move;
  }

  public double deadbandTurn(double turn) {
    if (Math.abs(turn) >= DriveConstants.deadbandX) {
      if (turn > 0) {
        turn = (turn - DriveConstants.deadbandX) / (1 - DriveConstants.deadbandX);
      } else {
        turn = (turn + DriveConstants.deadbandX) / (1 - DriveConstants.deadbandX);
      }
    } else {
      turn = 0;
    }
    return turn;
  }

  public void manualDrive(double move, double turn) {
    
    // If joysticks will prove to be too sensitive near the center, turn on the deadband driving
    
    // drive.arcadeDrive(deadbandMove(move), deadbandTurn(turn));
    // System.out.println("D X "+move + " Y " + turn);
    //drive.arcadeDrive(move, turn);
    drive.arcadeDrive(move, turn);
  }

  public void stopRobot() {
    leftDriveTalonFX[0].set(TalonFXControlMode.PercentOutput, 0);
    rightDriveTalonFX[0].set(TalonFXControlMode.PercentOutput, 0);
  }

  /** Get the number of tics moved by the left encoder */
  public double getLeftEncoder() {
    return leftDriveTalonFX[0].getSelectedSensorPosition();
  }

  /** Get the number of tics moved by the left encoder */
  public double getRightEncoder() {
    return rightDriveTalonFX[0].getSelectedSensorPosition();
  }

  // Make sure to adjust Units-of-measure if needed
  // The RAW output is "number of ticks per 100ms", so you may need to convert it
  // into m/s
  public double getLeftEncoderSpeed() {
    return leftDriveTalonFX[0].getSelectedSensorVelocity();
  }

  public double getRightEncoderSpeed() {
    return rightDriveTalonFX[0].getSelectedSensorVelocity();
  }

  public void zeroDriveEncoders() {  // zero encoders on master mmotor controllers of the drivetrain

    rightDriveTalonFX[0].setSelectedSensorPosition(0);
    leftDriveTalonFX[0].setSelectedSensorPosition(0);
  }

  // Configure encoders on primary motors
  public void configureEncoders(){
    leftDriveTalonFX[0].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    rightDriveTalonFX[0].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
  }

  //  **** TRAJECTORY DRIVING METHODS *****

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() { // needs to be meters per second
    return new DifferentialDriveWheelSpeeds(
        TranslateVelocityIntoMetersPerSecond(getLeftEncoderSpeed()),
        TranslateVelocityIntoMetersPerSecond(-getRightEncoderSpeed())
    );
  }

    /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {

    zeroDriveEncoders();   // Reset Encoders
    RobotContainer.pigeonIMUSubsystem.zeroHeading();  // Reset Yaw

    odometry.resetPosition(  // distances need to be in meters
        RobotContainer.pigeonIMUSubsystem.getRotation2d(),
        TranslateDistanceIntoMeters(getLeftEncoder()),
        TranslateDistanceIntoMeters(-getRightEncoder()),
        pose);
  }

  // Should be used in periodic when the trajectory navigation is running
  public void updateOdometry() {
    odometry.update(
      RobotContainer.pigeonIMUSubsystem.getRotation2d(),
      TranslateDistanceIntoMeters(getLeftEncoder()),
      TranslateDistanceIntoMeters(-getRightEncoder())
    );
  }

  public void updateTrajectoryOdometry() {
    odometry.update(
      RobotContainer.pigeonIMUSubsystem.getRotation2d(),
      TranslateDistanceIntoMeters(leftDriveTalonFX[0].getSelectedSensorPosition()),
      TranslateDistanceIntoMeters(-rightDriveTalonFX[0].getSelectedSensorPosition())
    ); 
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {

    //System.out.println("TV L:" + leftVolts + " R:" + rightVolts);

    leftDriveTalonFX[0].setVoltage(leftVolts);
    rightDriveTalonFX[0].setVoltage(-rightVolts);
    drive.feed();
  }

  public double TranslateDistanceIntoMeters(double distanceRawUnits) {
    return Units.inchesToMeters(distanceRawUnits / RobotDriveChassisConstants.tickPerInch) ;
  }

  public double TranslateVelocityIntoMetersPerSecond(double velocityRawUnits) {
    // Raw units - ticks per 100ms
    return Units.inchesToMeters((velocityRawUnits * 10) / RobotDriveChassisConstants.tickPerInch) ;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
