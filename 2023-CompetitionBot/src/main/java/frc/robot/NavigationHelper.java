package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.NavigationConstants;

public class NavigationHelper {
    public static Pose2d transformLeftCameraPoseToRobotPose(Pose2d cameraPose) {
        return cameraPose.exp(new Twist2d(
            -NavigationConstants.xOffsetOfCameraFromTurret,
            -NavigationConstants.yOffsetOfCameraFromTurret,
            -Units.degreesToRadians(NavigationConstants.angleOffsetOfCameraFromTurretDirection)
            ));
    }
    public static Pose2d transformRightCameraPoseToRobotPose(Pose2d cameraPose) {
        return cameraPose.exp(new Twist2d(
            -NavigationConstants.xOffsetOfCameraFromTurret,
            NavigationConstants.yOffsetOfCameraFromTurret,
            Units.degreesToRadians(NavigationConstants.angleOffsetOfCameraFromTurretDirection)
            ));
    }
}
