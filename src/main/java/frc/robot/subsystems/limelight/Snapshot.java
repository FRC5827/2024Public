package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose3d;

/** Snapshot of LimeLight values. */
public final class Snapshot {
    private final int m_tid;
    private final double m_tx;
    private final double m_ty;
    private final double m_ta;
    private final double m_tl;
    private final double m_cl;

    private final Pose3d m_botPose;
    private final Pose3d m_targetPoseCameraSpace;
    private final Pose3d m_targetPoseRobotSpace;
    private final Pose3d m_botPoseTargetSpace;

    Snapshot(int tid, double tx, double ty, double ta, double tl, double cl, Pose3d botPose, Pose3d targetPoseCameraSpace, Pose3d targetPoseRobotSpace, Pose3d botPoseTargetSpace) {
        m_tid = tid;
        m_tx = tx;
        m_ty = ty;
        m_ta = ta;
        m_tl = tl;
        m_cl = cl;
        m_botPose = botPose;
        m_targetPoseCameraSpace = targetPoseCameraSpace;
        m_targetPoseRobotSpace = targetPoseRobotSpace;
        m_botPoseTargetSpace = botPoseTargetSpace;
    }

    public int getTid() {
        return m_tid;
    }

    public double getTx() {
        return m_tx;
    }

    public double getTy() {
        return m_ty;
    }

    public double getTa() {
        return m_ta;
    }

    public double getTl() {
        return m_tl;
    }

    public double getCl() {
        return m_cl;
    }

    public double getLatencySeconds() {
        return (m_tl + m_cl) / 1000.0;
    }

    public Pose3d getBotPose() {
        return m_botPose;
    }

    public Pose3d getTargetPoseCameraSpace() {
        return m_targetPoseCameraSpace;
    }

    public Pose3d getBotPoseTargetSpace() {
        return m_botPoseTargetSpace;
    }

    public Pose3d getTargetPoseRobotSpace() {
        return m_targetPoseRobotSpace;
    }

    public AprilTags getTag() {
        return AprilTags.getTagForNumber(m_tid);
    }
}