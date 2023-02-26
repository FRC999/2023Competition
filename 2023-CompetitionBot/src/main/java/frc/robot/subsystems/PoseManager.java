// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.NavigationConstants;
import frc.robot.Constants.GamepieceManipulator.Arm;

/** Add your docs here. */
public class PoseManager {
    private ArrayList<Pose2d> poseQueue = new ArrayList<Pose2d>();

    /**
     * Delete all poses from the pose queue
     */
    public void clearAllPoses() {
        poseQueue.clear();
    }

    /**
     * Return number of poses in the pose queue
     * @return
     */
    public int numberOfPoses() {
        return poseQueue.size();
    }

    /**
     * Add pose to the pose queue
     */
    public void addPose(Pose2d pose2d) {
        if (poseQueue.size() >= NavigationConstants.POSE_QUEUE_MAXSIZE)
            poseQueue.remove(0);
        poseQueue.add(pose2d);
    } 

    /**
     * Eliminate outliers from the pose queue,
     * get average of X,Y and Angle from the remaining poses
     * Construct the pose consisting of averages and retuirn as a final pose
     * @return - pose based on averages without outliers
     */
    public Pose2d getPose() {

        if (poseQueue.size()<1) {  // If do not have a valid pose to return, return a dummy pose
            return NavigationConstants.dummyPose;
        }
        
        // Stream implementation of the AVERAGE calculation
        double xMean = poseQueue.stream()
                    .mapToDouble(Pose2d::getX)
                    .average()
                    .orElse(-1)
                    ;

        double yMean = poseQueue.stream()
                    .mapToDouble(Pose2d::getY)
                    .average()
                    .orElse(-1)
                    ;

        double zMean = poseQueue.stream()
                    .mapToDouble(p->p.getRotation().getDegrees())
                    .average()
                    .orElse(-1)
                    ;

        double xSD = Math.sqrt(poseQueue.stream()
                        .mapToDouble((p) -> Math.pow(p.getX() - xMean, 2))
                        .sum()/poseQueue.size());

        double ySD = Math.sqrt(poseQueue.stream()
                        .mapToDouble((p) -> Math.pow(p.getY() - yMean, 2))
                        .sum()/poseQueue.size());

        double zSD = Math.sqrt(poseQueue.stream()
                        .mapToDouble((p) -> Math.pow(p.getRotation().getDegrees() - zMean, 2))
                        .sum()/poseQueue.size());


        // Calculate the average pose - STREAMS implementation
        //List<Pose2d> returnPose =  new ArrayList<Pose2d>();
        double xFinal = poseQueue.stream()
            .filter(x -> Math.abs(x.getX() - xMean) <= NavigationConstants.SD_FACTOR*xSD )
            .filter(y -> Math.abs(y.getY() - yMean) <= NavigationConstants.SD_FACTOR*ySD )
            .filter(z -> Math.abs(z.getRotation().getDegrees() - zMean) <= Math.abs(NavigationConstants.SD_FACTOR*zSD) )
            .mapToDouble(Pose2d::getX)
            .average()
            .orElse(-1)
            ;

        double yFinal = poseQueue.stream()
            .filter(x -> Math.abs(x.getX() - xMean) <= NavigationConstants.SD_FACTOR*xSD )
            .filter(y -> Math.abs(y.getY() - yMean) <= NavigationConstants.SD_FACTOR*ySD )
            .filter(z -> Math.abs(z.getRotation().getDegrees() - zMean) <= Math.abs(NavigationConstants.SD_FACTOR*zSD) )
            .mapToDouble(Pose2d::getY)
            .average()
            .orElse(-1)
            ;

        double zFinal = poseQueue.stream()
            .filter(x -> Math.abs(x.getX() - xMean) <= NavigationConstants.SD_FACTOR*xSD )
            .filter(y -> Math.abs(y.getY() - yMean) <= NavigationConstants.SD_FACTOR*ySD )
            .filter(z -> Math.abs(z.getRotation().getDegrees() - zMean) <= Math.abs(NavigationConstants.SD_FACTOR*zSD) )
            .mapToDouble(p->p.getRotation().getDegrees())
            .average()
            .orElse(-1)
            ;

        return new Pose2d(xFinal, yFinal, new Rotation2d(Units.degreesToRadians(zFinal)));
    }

    public double[] getTargeting(Pose2d targetPose) {

        Pose2d robotPose = getPose();
        Transform2d targetTransform = robotPose.minus(targetPose);
        double relX = targetTransform.getTranslation().getX();
        double relY = targetTransform.getTranslation().getY();
        double targetDistance = Math.hypot(relX, relY);
        if(targetDistance > Arm.maximumExtension)
            targetDistance = -1;
        return (new double[]{targetTransform.getRotation().getDegrees(), targetDistance});
    }
}
