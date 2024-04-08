package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveDrive extends SubsystemBase {
    private final SwerveDriveOdometry m_odometry;

    private final SwerveModule[] m_swerveModules;
    private final TimeInterpolatableBuffer<Pose2d> m_poseBuffer;
    private final AHRS m_gyro;

    private int m_periodicCounter = 0;
    private Rotation2d m_fieldRelativeOffset = new Rotation2d();

    // The Field2d class shows the field in the sim GUI
    private final Field2d m_fieldSim;

    private final ShuffleboardTab m_driveShuffleboardTab = Shuffleboard.getTab("Drivetrain");
    private final GenericEntry m_xEntry = m_driveShuffleboardTab.add("X", 0.0)
            .withPosition(0, 2)
            .withSize(1, 1)
            .getEntry();
    private final GenericEntry m_yEntry = m_driveShuffleboardTab.add("Y", 0.0)
            .withPosition(0, 3)
            .withSize(1, 1)
            .getEntry();
    private final GenericEntry m_heading = m_driveShuffleboardTab.add("Heading", 0.0)
            .withPosition(0, 4)
            .withSize(1, 1)
            .getEntry();
    private final GenericEntry m_pose = m_driveShuffleboardTab.add("Pose", "No Data")
            .withPosition(2, 0)
            .withSize(4, 1)
            .getEntry();

    public SwerveDrive(AHRS gyro) {
        m_gyro = gyro;
        Pose2d initialPose = new Pose2d();

        m_swerveModules = new SwerveModule[] {
                new SwerveModule(SwerveConstants.FrontLeft.ModuleConstants, SwerveConstants.kCanBus),
                new SwerveModule(SwerveConstants.FrontRight.ModuleConstants, SwerveConstants.kCanBus),
                new SwerveModule(SwerveConstants.BackLeft.ModuleConstants, SwerveConstants.kCanBus),
                new SwerveModule(SwerveConstants.BackRight.ModuleConstants, SwerveConstants.kCanBus)
        };

        m_odometry = new SwerveDriveOdometry(SwerveConstants.Kinematics.kSwerveKinematics, m_gyro.getRotation2d(), getSwerveModulePositions(true), initialPose);
        m_poseBuffer = TimeInterpolatableBuffer.createBuffer(1.0);

        zeroHeading();

        m_fieldSim = new Field2d();
        SmartDashboard.putData("Field", m_fieldSim);

        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPoseAndGyroOffset, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            AutoConstants.pathFollowerConfig,
            AutoConstants::shouldFlipPath,
            this // Reference to this subsystem to set requirements
        );
    }

    public void drive(double forward, double strafe, double rotation, boolean fieldRelative, boolean slowMode,
            boolean isOpenLoop) {
        SmartDashboard.putNumber("/SwerveDrive/forward", forward);
        SmartDashboard.putNumber("/SwerveDrive/strafe", strafe);
        SmartDashboard.putNumber("/SwerveDrive/rotation", rotation);
        SmartDashboard.putBoolean("/SwerveDrive/fieldRelative", fieldRelative);
        SmartDashboard.putBoolean("/SwerveDrive/slowMode", slowMode);
        SmartDashboard.putBoolean("/SwerveDrive/isOpenLoop", isOpenLoop);

        if (slowMode) {
            forward *= SwerveConstants.kSlowModeFactor;
            strafe *= SwerveConstants.kSlowModeFactor;
            rotation *= SwerveConstants.kSlowModeFactor;
        }

        ChassisSpeeds chassisSpeeds = fieldRelative ? 
                                      ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getFieldRelativeAngle()):
                                      new ChassisSpeeds(forward, strafe, rotation);
        
        setChassisSpeeds(chassisSpeeds, isOpenLoop?DriveRequestType.OpenLoopVoltage:DriveRequestType.Velocity);
    }

    /**
     * Used for auton
     */
    public void setChassisSpeeds(ChassisSpeeds desiredSpeeds) {
        setChassisSpeeds(desiredSpeeds, DriveRequestType.OpenLoopVoltage);
    }

    private void setChassisSpeeds(ChassisSpeeds chassisSpeeds, DriveRequestType requestType) {
        SwerveModuleState[] desiredStates = SwerveConstants.Kinematics.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeed);
        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].apply(desiredStates[i], requestType);
        }
    }

    /**
     * Used by PathPlanner to get current robot speed
     * @return
     */
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.Kinematics.kSwerveKinematics.toChassisSpeeds(getStates());
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[m_swerveModules.length];
        for (int i = 0; i < m_swerveModules.length; i++) {
            states[i] = m_swerveModules[i].getCurrentState();
        }
        return states;
    }

    /**
     * Resets the pose and also field relative gyro offset
     */
    public void resetPoseAndGyroOffset(Pose2d newPose) {
        Rotation2d rotation = newPose.getRotation();
        if(AutoConstants.shouldFlipPath()) {
            rotation = GeometryUtil.flipFieldRotation(rotation);
        }
        zeroHeading();
        m_fieldRelativeOffset = m_fieldRelativeOffset.plus(rotation);
        resetPose(newPose);
    }

    /**
     * Resets the pose to the given pose
     */
    public void resetPose(Pose2d newPose) {
        m_odometry.resetPosition(m_gyro.getRotation2d(), getSwerveModulePositions(false), newPose);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public Pose2d getPose(double timestamp) {
        return m_poseBuffer.getSample(timestamp).orElseGet(this::getPose);
    }

    private SwerveModulePosition[] getSwerveModulePositions(boolean refresh) {
        SwerveModulePosition[] positions = new SwerveModulePosition[m_swerveModules.length];
        for(int i = 0; i < positions.length; i++) {
            positions[i] = m_swerveModules[i].getPosition(refresh);
        }
        return positions;
    }

    /**
     * Zeroes the heading of the robot.
     * This method also resets the pose so that it doesn't appear
     * that the robot has changed positon.
     */
    public void zeroHeading() {
        m_fieldRelativeOffset = m_gyro.getRotation2d().unaryMinus();
        resetPose(getPose());
    }

    public Rotation2d getFieldRelativeAngle() {
        return m_gyro.getRotation2d().plus(m_fieldRelativeOffset);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Y direction Tiltness: ", m_gyro.getPitch());
        SmartDashboard.putNumber("X direction Tiltness: ", m_gyro.getRoll());

        try {
            m_odometry.update(m_gyro.getRotation2d(), getSwerveModulePositions(true));
            m_poseBuffer.addSample(Timer.getFPGATimestamp(), getPose());
        } catch (ArithmeticException e) {
            System.out.println("Division by zero in SwerveDrive periodic updating odometry");
        }

        Pose2d currentPose = getPose();

        m_fieldSim.setRobotPose(currentPose);
        m_heading.setDouble(currentPose.getRotation().getDegrees());

        // pose string creation is relatively expensive, so only update stats occasionally
        if (m_periodicCounter % 15 == 0) {
            m_xEntry.setDouble(currentPose.getX());
            m_yEntry.setDouble(currentPose.getY());

            m_pose.setString(currentPose.toString());
        }

        m_periodicCounter++;
    }
}
