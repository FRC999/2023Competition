// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class PoseTester {

    static double[][] poseValues = 
        {
            {9.563,2.897, 30},
            {9.734,3.014, 32},
            {8.405,5.467, 28},
            {10.024,2.788, 39},
            {9.666,4.028, 30},
            {13.021,2.722, 27},
            {9.621,2.682, 31},
            {11.418,1.324, 25},
            {10.196,1.893, 29},
            {12.379,3.167, 31},
            {8.416,2.013, 34},
            {7.998,2.541, 28},
            {9.004,2.913, 30},
            {10.673,2.801, 36}
        };

    public static void main() {
        PoseManager poseManager = new PoseManager();
    

        for(int i =0; i<=14; i++) {
            Pose2d pose2d = new Pose2d(poseValues[i][0],poseValues[i][1],Rotation2d.fromDegrees(poseValues[i][2]));
            poseManager.addPose(pose2d);
        }

        Pose2d pose2d = poseManager.getPose();
        System.out.println(pose2d.getX() + "," + pose2d.getY() + "," + pose2d.getRotation().getDegrees());

    }
    
}
