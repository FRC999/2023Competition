// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

/*
 * This year we're not combining the code for multiple robots in the same project
 * So the constants will be declared as "final"
 */

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class RobotDriveChassisConstants { // configure the physical properties unique to the robot
    // here, such as dimensions, wheel diameter etc
    public static final double wheelDiameter = 5; // inches
    public static final double distanceBetweenWheels = 21.5; // inches
    public static final double chassisLength = 39; // inches, including bumpers
    public static final double chassisWidth = 30; // inches, including bumpers
    public static final int encoderUnitsPerMotorRotation = 2048;

    // TODO: check and change the constants below

    public static final double encoderGearReduction = 9.91748; // Confirmed with Kurt on 02/12/23
    public static final int encoderUnitsPerRobotAxelRotation = 20311; // measured manually on C2023 02/12/2023
    // guess, and should be measured)
    //franketbot code :)
    /*
    static final double clicksPerFoot = 1.021*(120615/10);
    public static final int tickPerInch = (int)(clicksPerFoot / 12); // (int) (2048/(4*Math.PI));
    */

    public static final double tickPerInch = 1292.0333; //Derived from calculation
    public static final double tolerance = 1*tickPerInch;
  }

  public static final class OIConstants {
    public static final int driverControllerPort = 1;
    public static final int turnControllerPort = 0;
  }

  public static final class DriveConstants {

    public static boolean isInvertdGearBox = false;

    public static int[] leftMotorPortID = new int[] { 1,2 };
    public static int[] rightMotorPortID = new int[] { 3,4 };

    public static int[] kLeftEncoderPorts = new int[] { 1 };
    public static int[] kRightEncoderPorts = new int[] { 3 };


    // Sensor phase - to ensure that sensor is positive when the output is positive
    public static boolean[] SensorPhase =  {false,false};
    // Invert motors
    public static boolean[] MotorInvert =  {true,false};

    public static boolean kLeftEncoderReversed = false;
    public static boolean kRightEncoderReversed = true;

    // TODO: Measure EncoderCPR - they're not attached to the wheelshaft, but rather to the Falcons
    public static final int kEncoderCPR = 1024;
    public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (RobotDriveChassisConstants.wheelDiameter * Math.PI) / (double) kEncoderCPR;

    // MotionMagic constants section

    // Closed loop constants

    //TODO: check and modify all MotionMagic constants

    /**
     * Talon FX supports multiple (cascaded) PID loops. For
     * now we just want the primary one.
     */
    public static final int kPIDLoopIdx = 0;

    // How long we wait for a configuration change to happen before we give up and
    // report a failure in milliseconds
    public final static int configureTimeoutMs = 30;

    public static int[] maximumLinearError;
    public static int[] maximumAngleError;
    
    // Full motor output value
    public final static int fullMotorOutput = 1023;
    // How many milliseconds between each closed loop call
    public final static int closedLoopPeriodMs = 1;
    // Motor neutral dead-band, set to the minimum 0.1%
    public final static double NeutralDeadband = 0.001;

    public static final int Izone_0 = 500;
    public static final double PeakOutput_0 = 1;

    /**
     * Talon PID methods often demand slot ID's, so we wil keep this here I do not
     * think we actually need it with Falcons anymore
     */
    public final static int SLOT_0 = 0;

    // Gains for MotionMagic
    public final static double motionMagicPidP_Value = 0.75;// * fullMotorOutput / encoderUnitsPerShaftRotation;
    //public final static double motionMagicPidP_Value = 0.2;
    public final static double motionMagicPidI_Value = 0.005;// * fullMotorOutput / encoderUnitsPerShaftRotation;
    //public final static double motionMagicPidI_Value = 0.0;
    public final static double motionMagicPidD_Value = 0.01;
    //public final static double motionMagicPidD_Value = 0.0;
    public final static double motionMagicPidF_Value = 2;
    //public final static double motionMagicPidF_Value = 0.2;

    public final static int motionMagicCruiseVelocity = 2250 * 3;
    public final static int motionMagicAcceleration = 2250 * 3;
    public final static int motionMagicSmoothing = 3;

    // Deadband values
    public final static double deadbandX = 0.1;
    public final static double deadbandY = 0.1;
    public final static double deadbandZ = 0.1;

    // The difference between the left and right side encoder values when the robot
    // is rotated 180 degrees
    // Allowable error to exit movement methods
    public static int defaultAcceptableError = 250;

    // Make smoother turns - see Cheezy Driving
    public static double turnAdjust = 0.6;

    //autoRoutineConstants

    //encoder ticks per foot
    public static int[] ticksPerFoot = new int[] {};

    //rotational ticks per degree
    public static int[] ticksPerDegree = new int[] {};

    // **** Software trajectory values; get from characterization
    /*public static final double ksVolts = 0.15219;
    public static final double kvVoltSecondsPerMeter = 2.1748;
    public static final double kaVoltSecondsSquaredPerMeter = 0.49391;
    */
    ///public static final double ksVolts = 0.1231;
    ///public static final double kvVoltSecondsPerMeter = 1.0882;
    ///public static final double kaVoltSecondsSquaredPerMeter = 0.092837;
    public static final double ksVolts = 0.1231;
    public static final double kvVoltSecondsPerMeter = 1.0882;
    public static final double kaVoltSecondsSquaredPerMeter = 0.092837;
  
    public static final DifferentialDriveKinematics kDriveKinematics = 
      new DifferentialDriveKinematics(
        Units.inchesToMeters(RobotDriveChassisConstants.distanceBetweenWheels)
      );
    //public static final double trajectoryRioPidP_Value = 0.054151 ;
    //public static final double trajectoryRioPidD_Value = 0;
    //public static final double trajectoryRioPidI_Value = 0;
    

    // aquired values from sysid, multiplied by 100 since original values were too small
    public static final double trajectoryRioPidP_Value = 5.19693;
    //public static final double trajectoryRioPidD_Value = 0.011828;
    public static final double trajectoryRioPidD_Value = 0;
    public static final double trajectoryRioPidI_Value = 0.04;


    // Default max values for trajectories - m/s and m/s^2
    //Max velocity is 12 feet per second (3.5 is converted to meters)
    public static final double maxVelocityDefault = 3;  //3.5, but was changed for testing purposes
    public static final double maxAccelerationDefault = 2;  //2, but was changed for testing purposes
    //Max Acceleration is based Alex's superior intelligence

    public static final class BigFoot {
      public static final int[] footSolenoidChannels = {4,5}; // First channel - DOWN, second - UP
    }

    public static final class SelfBalance {
      public static final double balancePitch = 0;  // target pitch to self-balance on a charger
      public static final double poorMaxClimbingPower = 0.12; //once I know I am on ramp (Phase-1), use that power to climb further (Phase-2)
      public static final double poorMaxClimbingPitch = 15; // Pitch from which we start to reduce the motor power (Phase-2)
      public static final double angleTolerance = 12.5; // Pitch where we stop the robot when the charger starts to balance (Phase-2)
      public static final double rampReachedPitch = 14.0; // Pitch indicating that I reached the ramp and ending Phase-1
    }

  }

  public static final class GamepieceManipulator {

    public static final class Turret {
      public static final int turretMotorID = 20;
      public static final int turretEncoderPort = 20;
      public static final boolean turretMotorInverted = true;
      public static final boolean turretSensorPhase = false;

      public static final double ticksPerDegree = 85.36 ; // Encoder ticks per 1 degree of rotation

      // *** PID ***

      public static final double turretAbsoluteZero = 857.0;
      public static final double turretAbsoluteZeroRollover = 4096;
      public static final double turretAbsoluteZeroClockwisePositionLimit = 3000;
      public static final int PID_Turret_Idx = 0; // 0 - closed loop; 1 - open loop
      public static final int turret_configureTimeoutMs = 30;
      public static final int turret_closedLoopPeriodMs = 30; // For loop on the talon with local sensor - 1ms
      public static final double turret_NeutralDeadband = 0.001;
      
      public static final int turret_SLOT_0 = 0;
      public static final double turret_kP = 1.7;
      public static final double turret_kI = 0.002;
      public static final double turret_kD = 0.01;
      public static final double turret_kF = 0;
      public static final double turret_Acceleration = 100; // raw sensor units per 100 ms per second
      public static final double turret_CruiseVelocity = 100; // raw sensor units per 100 ms
      public static final int turret_Smoothing = 3; // CurveStrength. 0 to use Trapezoidal Motion Profile. [1,8] for S-Curve (greater value yields greater smoothing).
      public static final double turret_DefaultAcceptableError = 1; // Sensor units
      public static final double turret_Izone = 500;
      public static final double turret_PeakOutput = 1; // Closed Loop peak output
      
      //manual drive constant
      public static final double TdefaultPowerManual = 0.3;

      public static final double turretCruisingPosition = 200.0; // Cruising position when we have a gamepiece

    }
    public static final class Elevator {
      public static final int elevatorMotorID = 21;
      public static final int elevatorEncoderPort = 21;
      public static final boolean elevatorMotorInverted = false;
      public static final boolean elevatorSensorPhase = false;

      // PID
      public static int PID_Elevator_Idx = 0; // 0 - closed loop; 1 - open loop
      public static int elevator_configureTimeoutMs = 30;
      //coppied from turret constants
      public static final double elevator_NeutralDeadband = 0.001;
      public static final int elevator_closedLoopPeriodMs = 30;
      
      public static final double elevatorAbsoluteZero = 0.0;
      //TO CHANGE - PID values copied from turret constants
      public static final int elevator_SLOT_0 = 0;
      public static final double elevator_kP = 1.5;
      public static final double elevator_kI = 0.0002;
      public static final double elevator_kD = 15;
      public static final double elevator_kF = 0;
      public static final double elevator_Acceleration = 50; // raw sensor units per 100 ms per second
      public static final double elevator_CruiseVelocity = 50; // raw sensor units per 100 ms
      public static final int elevator_Smoothing = 3; // CurveStrength. 0 to use Trapezoidal Motion Profile. [1,8] for S-Curve (greater value yields greater smoothing).
      public static final double elevator_DefaultAcceptableError = 1; // Sensor units
      public static final double elevator_Izone = 500;
      public static final double elevator_PeakOutput = 1; // Closed Loop peak output
      public static final double elevator_PIDTolerance = 100;

      //manual drive constant
      public static final double EdefaultPowerManual = 0.3;

      //elevator constants
      

      //will set them in encoder ticks later on
      public static enum coneHeights{
        LowCone(1000),  //TODO: Set manualy to the encoder value for hight
        MidCone(2000),
        HighCone(3000),
        Cruising(1200); // Cruising height - when we transport the gamepiece
        private double encoderSetting;
        coneHeights(int height) {
          this.encoderSetting = height;
        }
        public double getHeight() {
          return encoderSetting;
        }
      }

    }
    public static final class Arm {
      public static final int armMotorID = 22;
      public static final int armEncoderPort = 22;
      public static final boolean armMotorInverted = false;
      public static final boolean armSensorPhase = false;

      public static final double ticksPerMeter = 300 ; // Encoder ticks per 1 meter of extension
      public static final double maximumExtension = 2.5; // meters

      // PID
      public static final int PID_Arm_Idx = 0; // 0 - closed loop; 1 - open loop
      public static final int arm_configureTimeoutMs = 30;
      //coppied from turret constants
      public static final double arm_NeutralDeadband = 0.001;
      public static final int arm_closedLoopPeriodMs = 30;

      public static final double armAbsoluteZero = 0.0;
      //TO CHANGE - PID values copied from turret constants
      public static final int arm_SLOT_0 = 0;
      public static final double arm_kP = 1.5;
      public static final double arm_kI = 0.0002;
      public static final double arm_kD = 15;
      public static final double arm_kF = 0;
      public static final double arm_Acceleration = 50; // raw sensor units per 100 ms per second
      public static final double arm_CruiseVelocity = 50; // raw sensor units per 100 ms
      public static final int arm_Smoothing = 3; // CurveStrength. 0 to use Trapezoidal Motion Profile. [1,8] for S-Curve (greater value yields greater smoothing).
      public static final double arm_DefaultAcceptableError = 1; // Sensor units
      public static final double arm_Izone = 500;
      public static final double arm_PeakOutput = 1; // Closed Loop peak output

      //manual drive constant
      public static final double AdefaultPowerManual = 0.3;
      
      public static final double armCruisingPosition = 200.0;

    }
    public static final class Claw { 
      public static final int[] clawSolenoidChannels = {1,2};   // First channel to CLOSE, second to OPEN
      public static final int[] flipperSolenoidChannels = {3,4}; // First channel UP, second - DOWN
    }


  }

  public static final class PigeonIMUConstants {
    // CAN ID of Pigeon2
    public static final int pigeonIMUId = 11;
  }

  public static final class PneumaticsConstants {

    public static final int compressorCANID = 0;

  }

  public static final class NavigationConstants {
    public static final double[] fieldCenter = {8.725, 4.043};
    // X,Y coordinates of the tags from 0,0 in the blue/lower-left
    public static final double tags[][] = {
      {}, // 0
      {7.24310 + fieldCenter[0], -2.93659 + fieldCenter[1]}, // id 1
      {7.24310 + fieldCenter[0], -1.26019 + fieldCenter[1]}, // id 2
      {7.24310 + fieldCenter[0], 0.41621 + fieldCenter[1]}, // id 3
      {7.90832 + fieldCenter[0], 2.74161 + fieldCenter[1]}, // id 4
      {-7.90832 + fieldCenter[0], 2.74161 + fieldCenter[1]}, // id 5
      {-7.24310 + fieldCenter[0], 0.41621 + fieldCenter[1]}, // id 6
      {-7.24310 + fieldCenter[0], -1.26019 + fieldCenter[1]}, // id 7
      {-7.24310 + fieldCenter[0], -2.93659 + fieldCenter[1]} // id 8
    } ;
    public static List<Pose2d> leftTargets;
    public static List<Pose2d> rightTargets;

    public static final double yTargetOffset = tags[8][1] - 0.512;

    public static final double xTargetOffset[] = {
      1.1055-tags[8][0], // front/bottom offset
      0.653-tags[8][0], // middle offset
      0.238-tags[8][0] // top offset
    };

    public static double xOffsetOfCameraFromTurret = -0.1; // Offset of camera lens from the turret center of rotation
    public static double yOffsetOfCameraFromTurret = 0.2; // Offset of camera lens from the turret center of rotation
    public static double angleOffsetOfCameraFromTurretDirection = 30; // degrees; offset of camera direction from the arm direction

  }
  public static final class TargetConstants {
    public static final int[] targetPoses = {0,1,2};
  }

}
