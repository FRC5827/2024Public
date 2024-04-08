package frc.robot.subsystems.limelight;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;

public final class FiducialSnapshot {

    @JsonProperty("fID")
    public double fiducialID;

    @JsonProperty("fam")
    public String fiducialFamily;

    @JsonProperty("t6c_ts")
    private double[] cameraPose_TargetSpace;

    @JsonProperty("t6r_fs")
    private double[] robotPose_FieldSpace;

    @JsonProperty("t6r_ts")
    private double[] robotPose_TargetSpace;

    @JsonProperty("t6t_cs")
    private double[] targetPose_CameraSpace;

    @JsonProperty("t6t_rs")
    private double[] targetPose_RobotSpace;

    public Pose3d getCameraPoseTargetSpace()
    {
        return toPose3D(cameraPose_TargetSpace);
    }
    public Pose3d getRobotPoseFieldSpace()
    {
        return toPose3D(robotPose_FieldSpace);
    }
    public Pose3d getRobotPoseTargetSpace()
    {
        return toPose3D(robotPose_TargetSpace);
    }
    public Pose3d getTargetPoseCameraSpace()
    {
        return toPose3D(targetPose_CameraSpace);
    }
    public Pose3d getTargetPose_RobotSpace()
    {
        return toPose3D(targetPose_RobotSpace);
    }

    public Pose2d getCameraPoseTargetSpace2D()
    {
        return toPose2D(cameraPose_TargetSpace);
    }
    public Pose2d getRobotPoseFieldSpace2D()
    {
        return toPose2D(robotPose_FieldSpace);
    }
    public Pose2d getRobotPoseTargetSpace2D()
    {
        return toPose2D(robotPose_TargetSpace);
    }
    public Pose2d getTargetPoseCameraSpace2D()
    {
        return toPose2D(targetPose_CameraSpace);
    }
    public Pose2d getTargetPoseRobotSpace2D()
    {
        return toPose2D(targetPose_RobotSpace);
    }
    
    @JsonProperty("ta")
    public double ta;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("txp")
    public double tx_pixels;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("typ")
    public double ty_pixels;

    @JsonProperty("ts")
    public double ts;

    @JsonIgnore
    public long timestamp;

    public FiducialSnapshot() {
        cameraPose_TargetSpace = new double[6];
        robotPose_FieldSpace = new double[6];
        robotPose_TargetSpace = new double[6];
        targetPose_CameraSpace = new double[6];
        targetPose_RobotSpace = new double[6];
    }

    public boolean isStale() {
        // Consider anything older than 40ms as stale.
        return isStale(RobotController.getFPGATime() - Constants.LimeLightConstants.kDefaultLimeLightStalenessLimit);
    }
    
    public boolean isStale(long limitTimestamp) {
        return this.timestamp < limitTimestamp;
    }

    private static Pose3d toPose3D(double[] inData) {
        if(inData.length < 6) {
            System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }

        return new Pose3d(
        new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                    Units.degreesToRadians(inData[5])));
    }

    private static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            System.err.println("Bad LL 2D Pose Data!");
            return new Pose2d();
        }

        Translation2d tran2d = new Translation2d(inData[0], inData[1]);
        Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
        return new Pose2d(tran2d, r2d);
    }
}