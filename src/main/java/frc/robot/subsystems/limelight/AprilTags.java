package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.AprilTagConstants;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

import org.ejml.dense.row.misc.TransposeAlgs_DDRM;

/**
 * Represent an April Tag on the field.
 * Each tag also indicates whether it is in a red or a blue alliance area. There is also "snap points"
 * that each tag carries, which are the relative positions of where the robot should stop when targeting
 * the given April Tag.
 * 
 * The Snappoints Poses are represented as NWU coordinates where April Tag is at origin and 
 * +x is coming outside of an April Tag's front face.
 * 
 */
public enum AprilTags {

    // We expect the data to be in WPILib coordinate space
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    
    TAG1(1, Alliance.Blue, TargetType.Source, new Pose2d[]{
        new Pose2d(0.43, 0, Rotation2d.fromDegrees(180)),
        new Pose2d(0.43, -0.63, Rotation2d.fromDegrees(180)), // 21
        new Pose2d(0.43, -1.27, Rotation2d.fromDegrees(180))
    }),
  
    TAG2(2, Alliance.Blue, TargetType.Source, new Pose2d[]{
        new Pose2d(0.43, 0, Rotation2d.fromDegrees(180)),
        new Pose2d(0.43, 0.63, Rotation2d.fromDegrees(180)), // 21
        new Pose2d(0.43, 1.27, Rotation2d.fromDegrees(180))
    }),

    TAG3(3, Alliance.Red, TargetType.Speaker, new Pose2d[]{
        new Pose2d(1.3, -0.53, Rotation2d.fromDegrees(180)), // center of speaker
        new Pose2d(1.2, -1.5088, Rotation2d.fromDegrees(130)), // right side of speaker
        new Pose2d(1.1, 0.4, Rotation2d.fromDegrees(215)) // very left (almost right in front of april tag 3) theoretically should 0.4412 but does not work
    },
    new Pose2d(0.1, -56.515, Rotation2d.fromDegrees(180))),
  
    TAG4(4, Alliance.Red, TargetType.Speaker, new Pose2d[]{
        new Pose2d(1.3, 0.0, Rotation2d.fromDegrees(180)), // center of speaker
        new Pose2d(1.15, -1.0, Rotation2d.fromDegrees(135)), // right of speaker
        new Pose2d(1.1, 0.85, Rotation2d.fromDegrees(215))
    },
    new Pose2d(0.1, 0.0, Rotation2d.fromDegrees(180))),

    TAG5(5, Alliance.Red, TargetType.Amp, new Pose2d[]{
        new Pose2d(0.55, 0.0, Rotation2d.fromDegrees(180))}),
    TAG6(6, Alliance.Blue, TargetType.Amp, new Pose2d[]{
        new Pose2d(0.55, 0.0, Rotation2d.fromDegrees(180))}),
    TAG7(7, Alliance.Blue, TargetType.Speaker, new Pose2d[] {
        new Pose2d(1.3, 0.0, Rotation2d.fromDegrees(180)), // center of speaker
        new Pose2d(1.15, -1.0, Rotation2d.fromDegrees(135)), // right of speaker
        new Pose2d(1.1, 0.85, Rotation2d.fromDegrees(220))
        },
    new Pose2d(0.1, 0.0, Rotation2d.fromDegrees(180))),

    TAG8(8, Alliance.Blue, TargetType.Speaker, new Pose2d[]{
        new Pose2d(1.4, 0.53, Rotation2d.fromDegrees(180)),
        new Pose2d(1.35, 1.6, Rotation2d.fromDegrees(215)),
        new Pose2d(1.35, -0.40, Rotation2d.fromDegrees(130))
    },
    new Pose2d(0.1, 56.515, Rotation2d.fromDegrees(180))),

    TAG9(9, Alliance.Red, TargetType.Source, new Pose2d[]{
        new Pose2d(0.43, 0, Rotation2d.fromDegrees(180)),
        new Pose2d(0.43, -0.63, Rotation2d.fromDegrees(180)),
        new Pose2d(0.43, -1.27, Rotation2d.fromDegrees(180))
    }),
    TAG10(10, Alliance.Red, TargetType.Source, new Pose2d[]{
        new Pose2d(0.43, 0, Rotation2d.fromDegrees(180)),
        new Pose2d(0.43, 0.63, Rotation2d.fromDegrees(180)),
        new Pose2d(0.43, 1.27, Rotation2d.fromDegrees(180))
    }),

    TAG11(11, Alliance.Red, TargetType.Stage, new Pose2d[]{
        Constants.AprilTagConstants.kStageSnapPointCenter
    }),

    TAG12(12, Alliance.Red, TargetType.Stage, new Pose2d[]{        
        Constants.AprilTagConstants.kStageSnapPointCenter,
    }),

    TAG13(13, Alliance.Red, TargetType.Stage, new Pose2d[]{        
        Constants.AprilTagConstants.kStageSnapPointCenter,
    }),

    TAG14(14, Alliance.Blue, TargetType.Stage, new Pose2d[]{        
        Constants.AprilTagConstants.kStageSnapPointCenter,
    }),

    TAG15(15, Alliance.Blue, TargetType.Stage, new Pose2d[]{        
        Constants.AprilTagConstants.kStageSnapPointCenter,
    }),

    TAG16(16, Alliance.Blue, TargetType.Stage, new Pose2d[]{
        Constants.AprilTagConstants.kStageSnapPointCenter,
    });

    private final int m_tagID;
    private final Alliance m_alliance; // Red or Blue
    private final TargetType m_targetType;
    private final Pose2d[] m_snapPoses;
    private final Collection<Transform2d> m_snapTransforms;
    private final Transform2d m_speakerTransform;

    AprilTags(int tagID, Alliance alliance, TargetType targetType, Pose2d[] snapPoses) {
        this(tagID, alliance, targetType, snapPoses, new Pose2d());
    }

    AprilTags(int tagID, Alliance alliance, TargetType targetType, Pose2d[] snapPoses, Pose2d centerOfSpeaker) {
        this.m_tagID = tagID;
        this.m_alliance = alliance;
        this.m_targetType = targetType;
        this.m_snapPoses = snapPoses;

        var snapTransforms = new ArrayList<Transform2d>();
        for (var pose : snapPoses) {
            pose = new Pose2d(pose.getX() * AprilTagConstants.kAprilTagSnapPointXMultiplier, pose.getY() * AprilTagConstants.kAprilTagSnapPointYMultiplier, pose.getRotation());
            // The transform can be used to map the tag-relative snap points to robot relative locations on the field.
            snapTransforms.add(Conversions.TagSpace.asRobotSpaceTransform(pose));
        }

        this.m_snapTransforms = Collections.unmodifiableCollection(snapTransforms);
        this.m_speakerTransform = Conversions.TagSpace.asRobotSpaceTransform(centerOfSpeaker);
    }

    public static AprilTags getTagForNumber(int seenTag) {
        switch (seenTag) {
            case 1:
                return TAG1;
            case 2:
                return TAG2;
            case 3:
                return TAG3;
            case 4:
                return TAG4;
            case 5:
                return TAG5;
            case 6:
                return TAG6;
            case 7:
                return TAG7;
            case 8:
                return TAG8;
            case 9:
                return TAG9;
            case 10:
                return TAG10;
            case 11:
                return TAG11;
            case 12:
                return TAG12;
            case 13:
                return TAG13;
            case 14:
                return TAG14;
            case 15:
                return TAG15;
            case 16:
                return TAG16;
        }
        
        return null;
    }

    public Pose2d[] getSnapPoints() {
        return m_snapPoses;
    }

    public Alliance getAlliance() {
        return m_alliance;
    }

    public TargetType getTargetType() {
        return m_targetType;
    }

    public Collection<Transform2d> getSnapPointTransforms() {
        return this.m_snapTransforms;
    }

    public Transform2d getSpeakerTransform() {
        return this.m_speakerTransform;
    }

    public int getTagNum() {
        return m_tagID;
    }

    public boolean matchesAlliance() {
        return DriverStation.getAlliance().isPresent() && getAlliance() == DriverStation.getAlliance().get();
    }
}