// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PigeonIMUConstants;

public class PigeonIMUSubsystem extends SubsystemBase {
  /** Creates a new PidgeonIMUSubsystem. */
  private static WPI_Pigeon2 pigeon2;
  private static Pigeon2_Faults pigeonFaults = new Pigeon2_Faults();
  private double[] xyz = new double[3]; // so not to allocate one every time

  public PigeonIMUSubsystem() {
    pigeon2 = new WPI_Pigeon2(PigeonIMUConstants.pigeonIMUId);
  }

  /**
   * Gets the pitch of the robot (X axis rotation) (pitch is rotation around the
   * horizontal axis perpendicular to straight forward)
   * 
   * @return The pitch of the robot
   */
  public double getPitch() {
    double[] ypr = new double[3];
    pigeon2.getYawPitchRoll(ypr);
    return ypr[1];
  }

  /**
   * Gets the roll of the robot (Y axis rotation) (roll is the leaning around the
   * axis that goes straight forward)
   * 
   * @return
   */
  public double getRoll() {
    double[] ypr = new double[3];
    pigeon2.getYawPitchRoll(ypr);
    return ypr[2];
  }

  /**
   * Gets the yaw of the robot (Z axis rotation) (yaw is the direction that the
   * robot is facing around an axis that shoots straight up)
   * 
   * @return
   */
  public double getYaw() {
    double[] ypr = new double[3];
    pigeon2.getYawPitchRoll(ypr);
    return ypr[0];
  }

  /**
   * Zeroes the yaw of the robot
   * 
   * @return The previous yaw
   */
  public double zeroYaw() {
    double temporaryDouble = getYaw();
    pigeon2.setYaw(0);
    return temporaryDouble;
  }

  public double setYaw(double y) {
    double temporaryDouble = getYaw();
    pigeon2.setYaw(y);
    return temporaryDouble;
  }

  public Rotation2d getRotation2d() {
    return pigeon2.getRotation2d() ;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    pigeon2.setYaw(0);
    System.out.println("Yaw and Fused Heading set");
  }

    /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return pigeon2.getRotation2d().getDegrees();
  }

    /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -pigeon2.getRate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
