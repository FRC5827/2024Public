package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class TargetingShuffleboard {
        private final ShuffleboardTab m_driveShuffleboardTab = Shuffleboard.getTab("Targeting");

        private final GenericEntry m_hasCandidate = m_driveShuffleboardTab.add("HasCandidate", false)
            .withPosition(0, 0)
            .withSize(1, 1)
            .getEntry();
        
        private final GenericEntry m_candidateTag = m_driveShuffleboardTab.add("Candidate Tag", -1)
            .withPosition(1, 0)
            .withSize(1, 1)
            .getEntry();

        private final GenericEntry m_candidateSnaps = m_driveShuffleboardTab.add("Candidate Snaps", -1)
            .withPosition(2, 0)
            .withSize(1, 1)
            .getEntry();

        private final GenericEntry m_candidateType = m_driveShuffleboardTab.add("Candidate Type", "NA")
            .withPosition(3, 0)
            .withSize(1, 1)
            .getEntry();

        private final GenericEntry m_isTargetLocked = m_driveShuffleboardTab.add("Target Locked", false)
            .withPosition(0, 1)
            .withSize(1, 1)
            .getEntry();

        private final GenericEntry m_targetTag = m_driveShuffleboardTab.add("Target Tag", -1)
            .withPosition(1, 1)
            .withSize(1, 1)
            .getEntry();

        private final GenericEntry m_targetSnaps = m_driveShuffleboardTab.add("Target Snaps", -1)
            .withPosition(2, 1)
            .withSize(1, 1)
            .getEntry();

        private final GenericEntry m_targetType = m_driveShuffleboardTab.add("Target Type", "NA")
            .withPosition(3, 1)
            .withSize(1, 1)
            .getEntry();


        private final GenericEntry m_isAligned = m_driveShuffleboardTab.add("Is Aligned", false)
            .withPosition(0, 2)
            .withSize(1, 1)
            .getEntry();

        private final GenericEntry m_distanceToTarget = m_driveShuffleboardTab.add("Distance", -1)
            .withPosition(1, 2  )
            .withSize(1, 1)
            .getEntry();

        private final GenericEntry m_distanceToTargetGraph = m_driveShuffleboardTab.add("Distance Graph", -1)
            .withPosition(2, 2)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kGraph)
            .getEntry();

        private final GenericEntry m_angleToTarget = m_driveShuffleboardTab.add("Angle Distance", -1)
            .withPosition(4, 2)
            .withSize(1, 1)
            .getEntry();
        
        private final GenericEntry m_angleToTargetGraph = m_driveShuffleboardTab.add("Angle Distance Graph", -1)
            .withPosition(5, 2)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kGraph)
            .getEntry();

        private final GenericEntry m_targetPoseInRobotSpace = m_driveShuffleboardTab.add("Target pose in robot space", "NA")
            .withPosition(4, 0)
            .withSize(3, 1)
            .getEntry();


        public void setCandidate(AprilTags tag) {
                m_hasCandidate.setBoolean(true);
                m_candidateTag.setInteger(tag.getTagNum());
                m_candidateSnaps.setInteger(tag.getSnapPoints().length);
                m_candidateType.setString(tag.getTargetType().name());
        }

        public void setTargetPoseInRobotSpace(Pose2d pose) {
                m_targetPoseInRobotSpace.setString(pose.toString());
        }

        public void clearCandidate() {
                m_hasCandidate.setBoolean(false);
                m_candidateTag.setInteger(-1);
                m_candidateSnaps.setInteger(-1);
                m_candidateType.setString("NA");

                m_targetPoseInRobotSpace.setString("NA");
        }

        public void setLocked(AprilTags tag) {
                m_isTargetLocked.setBoolean(true);
                m_targetTag.setInteger(tag.getTagNum());
                m_targetSnaps.setInteger(tag.getSnapPoints().length);
                m_targetType.setString(tag.getTargetType().name());
        }

        public void clearLocked() {
                m_isTargetLocked.setBoolean(false);
                m_targetTag.setInteger(-1);
                m_targetSnaps.setInteger(-1);
                m_targetType.setString("NA");
                m_isAligned.setBoolean(false);
        }

        public void setTargetAlignmentDelta(Pose2d delta) {
                var distance = delta.getTranslation().getNorm();
                m_distanceToTarget.setDouble(distance);
                m_distanceToTargetGraph.setDouble(distance);
                m_angleToTarget.setDouble(delta.getRotation().getDegrees());
                m_angleToTargetGraph.setDouble(delta.getRotation().getDegrees());
        }

        public void setTargetAligned(boolean value) {
                m_isAligned.setBoolean(value);
        }

        public void clearAlignmentData() {
                m_distanceToTarget.setDouble(-1);
                m_angleToTargetGraph.setDouble(-1);
                m_angleToTarget.setDouble(-1);
                m_angleToTarget.setDouble(-1);
                m_isAligned.setBoolean(false);
        }
}
