// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.NavigationConstants;

/** Add your docs here. */
public class PoseManager {
    private ArrayList<Pose2d> poseQueue = new ArrayList<Pose2d>();

    // Delete all poses from the queue 
    public void clearAllPoses() {
        poseQueue.clear();
    }

    public int numberOfPoses() {
        return poseQueue.size();
    }

    public void addPose(Pose2d pose2d) {
        if (poseQueue.size() >= NavigationConstants.POSE_QUEUE_MAXSIZE)
            poseQueue.remove(0);
        poseQueue.add(pose2d);
    } 

    public Pose2d getPose() {
        int queueSize = poseQueue.size();

        if (poseQueue.size()<1) {  // If do not have a valid pose to return, return a dummy pose
            return NavigationConstants.dummyPose;
        }

        double xSum=0, ySum=0, zSum=0, xMean=0, yMean=0, zMean=0, xSD=0, ySD=0, zSD=0;
        for(int i=0; i<queueSize; i++){
            Pose2d pose2d = poseQueue.get(i);
            xSum += pose2d.getX();
            ySum += pose2d.getY();
            zSum += pose2d.getRotation().getDegrees();
        }

        xMean = xSum/queueSize;
        yMean = ySum/queueSize;
        zMean = zSum/queueSize;
        xSum = 0;
        ySum = 0;
        zSum = 0;

        // Stream implementation of the AVERAGE calculation
        xMean = poseQueue.stream()
                    .mapToDouble(Pose2d::getX)
                    .average()
                    .orElse(-1)
                    ;

        yMean = poseQueue.stream()
                    .mapToDouble(Pose2d::getY)
                    .average()
                    .orElse(-1)
                    ;

        zMean = poseQueue.stream()
                    .mapToDouble(p->p.getRotation().getDegrees())
                    .average()
                    .orElse(-1)
                    ;

        //poseQueue.stream().filter(p->p.getX()>10).toList();

        /*
        for(int i=0; i<queueSize; i++){
            Pose2d pose2d = poseQueue.get(i);
            xSum += Math.pow(pose2d.getX()-xMean, 2);
            ySum += Math.pow(pose2d.getY()-yMean ,2);
            zSum += Math.pow(pose2d.getRotation().getDegrees()-zMean ,2);
        }
        xSD = Math.sqrt(xSum/queueSize);
        ySD = Math.sqrt(ySum/queueSize);
        zSD = Math.sqrt(zSum/queueSize);

        xSum = 0;
        ySum = 0;
        zSum = 0;
*/
        int validCount = 0;

        for(int i=0; i<queueSize; i++){
            Pose2d pose2d = poseQueue.get(i);
            if ((Math.abs(pose2d.getX()-xMean)<=NavigationConstants.MEAN_DEV*xMean) && 
                    (Math.abs(pose2d.getY()-yMean)<=NavigationConstants.MEAN_DEV*yMean) &&
                    (Math.abs(pose2d.getRotation().getDegrees()-zMean)<=NavigationConstants.MEAN_DEV*zMean)) {
                validCount++;
                xSum += pose2d.getX();
                ySum += pose2d.getY();
                zSum += pose2d.getRotation().getDegrees();
            }
        }
        
        xMean = xSum/validCount;
        yMean = ySum/validCount;
        zMean = zSum/validCount;

        Pose2d pose2d = new Pose2d(xMean, yMean, Rotation2d.fromDegrees(zMean));
        return pose2d;
    }
}
