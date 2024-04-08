package frc.robot.subsystems.limelight;

import java.util.ArrayList;
import java.util.Collection;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Targeting allows tracking of a main AprilTag as a "candidate" target. The system allows "locking" on
 * to the target, which means even if the system sees a different tag or loses track of the tag, it will still
 * operate based on the previously-locked-on target.
 */
public final class Targeting extends SubsystemBase {
    private static final Pose2d kOrigin = new Pose2d();

    private final LimeLight m_limeLight;
    private final Function<Double, Pose2d> m_robotPoseSupplier;
    private final CommandXboxController m_controller;
    private final TargetingShuffleboard m_shuffleboard = new TargetingShuffleboard();

    private Snapshot m_candidate;

    private boolean m_targetLocked;
    private Snapshot m_target;
    private Pose2d m_targetPoseRobotRelative;
    private Pose2d m_targetPose;
    private Pose2d m_deltaPose;
    private Pose2d m_speakerPose;
    private Pose2d m_speakerDeltaPose;

    public Targeting(LimeLight limeLight, CommandXboxController controller, Function<Double, Pose2d> robotPoseSupplier) {
        this.m_limeLight = limeLight;
        this.m_controller = controller;
        m_robotPoseSupplier = robotPoseSupplier;
    }

    /**
     * Lock the current candidate target. So long as the target is locked, the system will use its
     * data to track the position of a chosen target. 
     */
    public boolean lockTarget() {
        if (m_targetLocked || m_target != null) {
            m_targetLocked = true;
            m_shuffleboard.setLocked(m_target.getTag());
            return true;
        }

        return false;
    }

    /**
     * Release the target lock. A new tag can now be relocked.
     */
    public void releaseTarget() {
        m_targetLocked = false;
        m_shuffleboard.clearLocked();
    }

    /**
     * Gets the delta difference between where the robot is, and the locked-in target pose.
     */
    public Pose2d getDeltaPose() {
        return m_deltaPose;
    }

    public Pose2d getSpeakerDeltaPose() {
        return m_speakerDeltaPose;
    }
    
    /**
     * Return if a target is locked.
     */
    public boolean isTargetLocked() {
        return m_targetLocked;
    }

    /**
     * Returns if there is an available target for locking.
     */
    public boolean hasCandidateTarget() {
        return m_candidate != null;
    }

    /**
     * Set whether the robot is aligned to the targeted position.
     */
    public void setAligned(boolean aligned) {
        m_shuffleboard.setTargetAligned(aligned);
    }
    
    @Override
    public void periodic() {
        // Get a new Limelight data snapshot and check if it is a usable one (i.e. it matches our alliance).  
        var snapshot = m_limeLight.getSnapshot();
        if (snapshot == null || !snapshot.getTag().matchesAlliance()) {
            m_candidate = null;
            m_shuffleboard.clearCandidate();
            setRumble(false);
        }
        else {
            m_candidate = snapshot;
            m_shuffleboard.setCandidate(m_candidate.getTag());
            setRumble(!m_targetLocked);
        }
        
        // keep the targeting the data as the same as candidate data (for code simplicity) if there is no target locked.
        // Otherwise, check if the new data is about the locked target. If so, we can use it to re-adjust our target position.
        boolean adjustTarget = false;

        if (!m_targetLocked || (m_candidate != null && m_target.getTid() == m_candidate.getTid())) {
            m_target = m_candidate;
            adjustTarget = true;
        }

        if (m_target != null) {

            // Targeting data has changed. Recalculate the relevant snap point positions and selection of the snap point.
            if (adjustTarget) {
                // Pose of the tag, relative to the robot.
                var robotRelativeTagPose = Conversions.Limelight.CameraSpace.toNWU(m_target.getTargetPoseRobotSpace());
                m_shuffleboard.setTargetPoseInRobotSpace(robotRelativeTagPose);

                // Pose of snap points of the tag, relative to the robot.
                var snapPoints = getRobotRelativeSnapPoints(m_target.getTag().getSnapPointTransforms(), robotRelativeTagPose);

                var speakerCenter = robotRelativeTagPose.transformBy(m_target.getTag().getSpeakerTransform());

                // origin represents the robot. Find the nearest snap point.
                Pose2d nearestSnapPoint = kOrigin.nearest(snapPoints);

                m_targetPoseRobotRelative = nearestSnapPoint;
    
                // Calculate the transform from Robot to target position.
                // This is used to translate the location of the target to odometer-based coordinate system.
                var transform = m_targetPoseRobotRelative.minus(kOrigin);
                var poseAtSnapshot = m_robotPoseSupplier.apply(Timer.getFPGATimestamp() - snapshot.getLatencySeconds());
                m_targetPose = poseAtSnapshot.transformBy(transform);
                transform = speakerCenter.minus(kOrigin);
                m_speakerPose = poseAtSnapshot.transformBy(transform);
            }

            var robotPose = m_robotPoseSupplier.apply(Timer.getFPGATimestamp());
            
            // Calculate the delta from the current robot pose, to the desired/target robot pose.
            // This is used for driving to the target pose.
            m_deltaPose = m_targetPose.relativeTo(robotPose);
            m_shuffleboard.setTargetAlignmentDelta(m_deltaPose);
            m_speakerDeltaPose = m_speakerPose.relativeTo(robotPose);
        } else {
            m_targetPoseRobotRelative = null;
            m_targetPose = null;
            m_deltaPose = null;
            m_speakerPose = null;
            m_speakerDeltaPose = null;
            m_shuffleboard.clearAlignmentData();
        }
    }

    private void setRumble(boolean enable) {
        if (DriverStation.isTeleopEnabled() && enable) {
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 1);
        } else {
            m_controller.getHID().setRumble(RumbleType.kBothRumble, 0);
        }
    }

    /**
     * Given the snap point transforms (transforming from tag-at-origin to snap point) to robot relative coordinates by applying the 
     * transform to tag's robot-relative pose.
     */
    private static ArrayList<Pose2d> getRobotRelativeSnapPoints(Collection<Transform2d> snapPointTransforms, Pose2d robotRelativeTagPose) {
        // Find the pose of all snap points relative to the robot space.
        var snapPoses = new ArrayList<Pose2d>(snapPointTransforms.size());
        for (var transform : snapPointTransforms) {
            snapPoses.add(robotRelativeTagPose.transformBy(transform));
        }

        return snapPoses;
    }

    public TargetType getLockedTargetType() {
        if (m_target == null) {
            return TargetType.None;
        }

        return m_target.getTag().getTargetType();
    }
}
