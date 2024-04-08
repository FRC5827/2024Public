package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Conversion of coordinates between different coordinate systems and coordinate spaces.
 */
public final class Conversions {

    // Limelight uses multiple different coordinate systems.
    // https://docs.limelightvision.io/docs/docs-limelight/pipe1line-apriltag/apriltag-coordinate-systems

    // On the other hand, WPILib uses a different coordinate system:
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html

    // This code standardizes on WPILib coordinates (+x is "forward", +y is "left"). The first task is to convert
    // from Limelight coordinates to WPILib coordinates. Then, we can use the pose estimator to track the robot for
    // for this target.
    public static class Limelight {

        public static Pose3d asPose3d(double[] values) {
            var translation3d = new Translation3d(
                values[0],
                values[1],
                values[2]);
            var rotation3d = new Rotation3d(
                Units.degreesToRadians(values[3]),
                Units.degreesToRadians(values[4]),
                Units.degreesToRadians(values[5]));

            return new Pose3d(translation3d, rotation3d);        
        }

        public static class CameraSpace {
            /**
             * Converts a Limelight 3D camera space pose to 2D pose in NWU coordinate system.
             * @param robotSpacePose Limelight pose in 3D robot space
             * @return 2D pose in WPI coordinate system
             */
            public static Pose2d toNWU(Pose3d pose3d) {
                return new Pose2d(
                    pose3d.getZ(),
                    - pose3d.getX(),
                    Rotation2d.fromRadians(-pose3d.getRotation().getY()));
            }
        }
    }

    /**
     * Tag space is a made-up coordinate system. That is NWU based, where +x points out of
     * April Tag surface. It is useful for tracking coordinates that are relative to an April Tag.
     */
    public static class TagSpace {
        private static final Pose2d kTagPosition = new Pose2d(); // tag is at origin
        private static final Rotation2d k180 = Rotation2d.fromDegrees(180);

        /**
         * Converts a Tag-relative snap point Pose2d into a Transform2d that can be used 
         * to quickly calculate the Robot Space position of the snap point, given an April Tag's
         * position in Robot Space.
         * @param snapPose
         * @return
         */
        public static Transform2d asRobotSpaceTransform(Pose2d snapPose) {
            // Rotate the snap pose 180 degrees. Snap points are in an NWU 
            // coordinate that where +x is coming out of the tag. However,
            // the LimeLight provided tag position keeps the tag Pose as
            // in the reverse direction. 
            return new Transform2d(kTagPosition, snapPose.rotateBy(k180));
        }
    }
}
