package frc.robot.subsystems.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * this class uses the network table to keep track of where the robot is.
 * 
 *  https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
 */
public final class LimeLight extends SubsystemBase {
    private static final double[] kSentinelDoubleArray = new double[0];

    private final JSONParser m_jsonparser = new JSONParser();
    private final NetworkTable m_table;
    private final NetworkTableEntry m_tid;
    private final NetworkTableEntry m_tv;
    private final NetworkTableEntry m_tx;
    private final NetworkTableEntry m_ty;
    private final NetworkTableEntry m_ta;
    private final NetworkTableEntry m_tl;
    private final NetworkTableEntry m_cl;
    private final NetworkTableEntry m_botpose;
    private final NetworkTableEntry m_targetPoseCameraSpace;
    private final NetworkTableEntry m_targetPoseRobotSpace;
    private final NetworkTableEntry m_botPoseTargetSpace;
    private final NetworkTableEntry m_json;
    private final FiducialSnapshot[] m_fiducialSnapshots = new FiducialSnapshot[16];

    private Snapshot m_snapshot;
    private int m_staleness = 0;
    private String m_jsonSnapshot;
    private int m_jsonStaleness = 0;

    public LimeLight() {
        this.m_table = NetworkTableInstance.getDefault().getTable("limelight");
        this.m_tv = m_table.getEntry("tv");
        this.m_tid = m_table.getEntry("tid");
        this.m_tx = m_table.getEntry("tx");
        this.m_ty = m_table.getEntry("ty");
        this.m_ta = m_table.getEntry("ta");
        this.m_tl = m_table.getEntry("tl");
        this.m_cl = m_table.getEntry("cl");
        this.m_botpose = m_table.getEntry("botpose");
        this.m_targetPoseCameraSpace = m_table.getEntry("targetpose_cameraspace");
        this.m_targetPoseRobotSpace = m_table.getEntry("targetpose_robotspace");
        this.m_botPoseTargetSpace = m_table.getEntry("botpose_targetspace");
        this.m_json = m_table.getEntry("json");
    }

    @Override
    public void periodic() {
        snapshot();        
    }

    private void snapshot() {
        var tv = m_tv.getInteger(0) != 0;
        m_staleness++;     
        if (!tv) {
            // TODO: cleanup staleness system.
            if (m_staleness > 2) {
                m_snapshot = null;
            }
            return;
        }

        // Filter out any non-recognized tags. For 2024 Crescendo game, the tags are between 1 and 16.
        var tid = (int)m_tid.getInteger(-1);
        if (tid < 1 || tid > 16) {
            return;
        }

        var tx = m_tx.getDouble(Double.MIN_VALUE);
        if (tx == Double.MIN_VALUE) {
            return;
        }

        var ty = m_ty.getDouble(Double.MIN_VALUE);
        if (ty == Double.MIN_VALUE) {
            return;
        }

        var ta = m_ta.getDouble(Double.MIN_VALUE);
        if (ta == Double.MIN_VALUE) {
            return;
        }
        
        var tl = m_tl.getDouble(Double.MIN_VALUE);
        if (tl == Double.MIN_VALUE) {
            return;
        }

        var cl = m_cl.getDouble(Double.MIN_VALUE);
        if(cl == Double.MIN_VALUE) {
            return;
        }

        var botpose3dArray = m_botpose.getDoubleArray(kSentinelDoubleArray);
        if (botpose3dArray == kSentinelDoubleArray) {
            return;
        }
        var botPose3d = Conversions.Limelight.asPose3d(botpose3dArray);

        var targetPoseCameraSpaceArray = m_targetPoseCameraSpace.getDoubleArray(kSentinelDoubleArray);
        if (targetPoseCameraSpaceArray == kSentinelDoubleArray) {
            return;
        }
        var targetPoseCameraSpace3d = Conversions.Limelight.asPose3d(targetPoseCameraSpaceArray);

        var targetPoseRobotSpaceArray = m_targetPoseRobotSpace.getDoubleArray(kSentinelDoubleArray);
        if (targetPoseRobotSpaceArray == kSentinelDoubleArray) {
            return;
        }
        var targetPoseRobotSpace3d = Conversions.Limelight.asPose3d(targetPoseRobotSpaceArray);

        var botPoseTargetSpace3dArray = m_botPoseTargetSpace.getDoubleArray(kSentinelDoubleArray);
        if (botPoseTargetSpace3dArray == kSentinelDoubleArray) {
            return;
        }
        var botPoseTargetSpace3d = Conversions.Limelight.asPose3d(botPoseTargetSpace3dArray);

        m_staleness = 0;
        this.m_snapshot = new Snapshot(tid, tx, ty, ta, tl, cl, botPose3d, targetPoseCameraSpace3d, targetPoseRobotSpace3d, botPoseTargetSpace3d);
    }

    public Snapshot getSnapshot() {
        return m_snapshot;
    }

    public Snapshot getLatestSnapshot() {
        if (m_staleness < 2) {
            return m_snapshot;
        }
        return null;
    }

    public FiducialSnapshot getFiducialSnapshot(int id) {
        return m_fiducialSnapshots[id - 1];
    }

    public boolean hasNonStaleFudicial(int id) {
        var fiducial = getFiducialSnapshot(id);
        return fiducial != null && !fiducial.isStale();
    }
}