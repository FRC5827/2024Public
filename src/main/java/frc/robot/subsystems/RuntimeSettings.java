package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoFireConstants;
import frc.robot.Constants.OperatorConstants;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class RuntimeSettings extends SubsystemBase {

    // Swerve Drive configuration
    private final NetworkTableEntry m_fieldRelativeStateShuffleboard;
    private final NetworkTableEntry m_fieldRelativeStatusShuffleboard;
    private final NetworkTableEntry m_slowModeStateShuffleboard;
    private final NetworkTableEntry m_slowModeStatusShuffleboard;
    private final ShuffleboardTab m_driveShuffleboardTab;
    private final NetworkTableEntry m_autoFireRadiusShuffleboard;
    private final NetworkTableEntry m_autoFireDistanceToleranceShuffleboard;
    private final NetworkTableEntry m_autoFireAngleToleranceShuffleboard;

    private boolean m_fieldRelative;
    private boolean m_slowMode;
    private double m_autoFireRadius;
    private double m_autoFireAngleTolerance;
    private double  m_autoFireDistanceTolerance;
    
    public RuntimeSettings() {
            // button for toggle and status indicator
        m_fieldRelativeStateShuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard")
            .getEntry("Drivetrain/Field Relative");
        m_fieldRelativeStatusShuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard")
            .getEntry("Drivetrain/FieldRelativeStatus");
        m_slowModeStateShuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard")
            .getEntry("Drivetrain/Slow Mode");
        m_slowModeStatusShuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard")
            .getEntry("Drivetrain/SlowModeStatus");
        m_autoFireAngleToleranceShuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard")
            .getEntry("Drivetrain/AutoFireAngleTolerance");
        m_autoFireDistanceToleranceShuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard")
            .getEntry("Drivetrain/AutoFireDistanceTolerance");
        m_autoFireRadiusShuffleboard = NetworkTableInstance.getDefault().getTable("Shuffleboard")
            .getEntry("Drivetrain/AutoFireRadius");

        m_fieldRelative = m_fieldRelativeStateShuffleboard.getBoolean(OperatorConstants.kDefaultSlowMode);

        m_slowMode = OperatorConstants.kDefaultSlowMode;

        m_autoFireAngleTolerance = m_autoFireAngleToleranceShuffleboard.getDouble(AutoFireConstants.kAutoFireAngleTolerance);
        m_autoFireRadius = m_autoFireRadiusShuffleboard.getDouble(AutoFireConstants.kAutoFireDistance);
        m_autoFireDistanceTolerance = m_autoFireDistanceToleranceShuffleboard.getDouble(AutoFireConstants.kAutoFireTolerance);

        m_driveShuffleboardTab = Shuffleboard.getTab("Drivetrain");
        m_driveShuffleboardTab.add("Field Relative", OperatorConstants.kDefaultFieldRelative)
                                                    .withPosition(0, 0)
                                                    .withSize(1, 1)
                                                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
        m_driveShuffleboardTab.add("FieldRelativeStatus", OperatorConstants.kDefaultFieldRelative)
                                                    .withPosition(1, 0)
                                                    .withSize(1, 1)
                                                    .withWidget(BuiltInWidgets.kBooleanBox);
        m_driveShuffleboardTab.add("Slow Mode", OperatorConstants.kDefaultSlowMode)
                                                    .withPosition(0, 1)
                                                    .withSize(1, 1)
                                                    .withWidget(BuiltInWidgets.kToggleButton).getEntry();
        m_driveShuffleboardTab.add("SlowModeStatus", OperatorConstants.kDefaultSlowMode)
                                                    .withPosition(1, 1)
                                                    .withSize(1, 1)
                                                    .withWidget(BuiltInWidgets.kBooleanBox);
        m_driveShuffleboardTab.add("AutoFireRadius", AutoFireConstants.kAutoFireDistance)
                                                    .withPosition(2, 2)
                                                    .withSize(1, 1)
                                                    .getEntry();
        m_driveShuffleboardTab.add("AutoFireDistanceTolerance", AutoFireConstants.kAutoFireTolerance)
                                                    .withPosition(3, 2)
                                                    .withSize(1, 1)
                                                    .getEntry();
        m_driveShuffleboardTab.add("AutoFireAngleTolerance", AutoFireConstants.kAutoFireAngleTolerance)
                                                    .withPosition(4, 2)
                                                    .withSize(1, 1)
                                                    .getEntry();
    }

    public boolean isFieldRelative() {
        return m_fieldRelative;
    }

    public void setFieldRelative(boolean value) {
        m_fieldRelative = value;
        if (m_fieldRelativeStateShuffleboard != null) {
            m_fieldRelativeStateShuffleboard.setBoolean(value);
        }
    }

    public boolean isSlowMode() {
        return m_slowMode;
    }

    public void setSlowMode(boolean value) {
        m_slowMode = value;
        if (m_slowModeStateShuffleboard != null) {
            m_slowModeStateShuffleboard.setBoolean(value);
        }
    }

    public void setAutoFireRadius(double value) {
        m_autoFireRadius = value;

        if (m_autoFireRadiusShuffleboard != null) {
            m_autoFireRadiusShuffleboard.setDouble(value);
        }
    }

    public double getAutoFireRadius() {
        return m_autoFireRadius;
    }

    public double getAutoFireDistanceTolerance() {
        return m_autoFireDistanceTolerance;
    }

    public double getAutoFireAngleTolerance() {
        return m_autoFireAngleTolerance;
    }

    public void setAutoFireDistanceTolerance(double value) {
        m_autoFireRadius = value;

        if (m_autoFireDistanceToleranceShuffleboard != null) {
            m_autoFireDistanceToleranceShuffleboard.setDouble(value);
        }
    }

    public void setAutoFireAngleTolerance(double value) {
        m_autoFireRadius = value;

        if (m_autoFireAngleToleranceShuffleboard != null) {
            m_autoFireAngleToleranceShuffleboard.setDouble(value);
        }
    }

    @Override
    public void periodic() {
        if (m_fieldRelativeStateShuffleboard != null) {
            boolean dashboardSwitch = m_fieldRelativeStateShuffleboard.getBoolean(m_fieldRelative);
            m_fieldRelative = dashboardSwitch;
        }

        if (m_slowModeStateShuffleboard != null) {
            boolean dashboardSwitch = m_slowModeStateShuffleboard.getBoolean(m_slowMode);
            m_slowMode = dashboardSwitch; 
        }

        // update current field relative status on Shuffleboard
        if (m_fieldRelativeStatusShuffleboard != null) {
            m_fieldRelativeStatusShuffleboard.setBoolean(m_fieldRelative);
        }

        // update current slowMode status on Shuffleboard
        if (m_slowModeStatusShuffleboard != null) {
            m_slowModeStatusShuffleboard.setBoolean(m_slowMode);
        }

        if (m_autoFireRadiusShuffleboard != null) {
            m_autoFireRadius = m_autoFireRadiusShuffleboard.getDouble(m_autoFireRadius);
        }

        if (m_autoFireAngleToleranceShuffleboard != null) {
            m_autoFireAngleTolerance = m_autoFireAngleToleranceShuffleboard.getDouble(m_autoFireAngleTolerance);
        }

        if (m_autoFireDistanceToleranceShuffleboard != null) {
            m_autoFireDistanceTolerance = m_autoFireDistanceToleranceShuffleboard.getDouble(m_autoFireDistanceTolerance);
        }
    }
}