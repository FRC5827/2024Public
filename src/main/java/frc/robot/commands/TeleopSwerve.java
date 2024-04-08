// Copyright (c) Code Purple - Team 5827, FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.RuntimeSettings;
import frc.robot.subsystems.SwerveDrive;

public class TeleopSwerve extends Command {

    private final boolean m_openLoop;
    
    private final SwerveDrive m_swerveDrive;
    private final RuntimeSettings m_settings;

    private final AxisGetter m_translationAxis;
    private final AxisGetter m_strafeAxis;
    private final AxisGetter m_rotationAxis;

    private final boolean m_alwaysSlowMode;

    public static interface AxisGetter {
        public double getAxisValue();
    }

    // Driver control
    public TeleopSwerve(SwerveDrive swerveDrive, RuntimeSettings settings, AxisGetter translationAxis, AxisGetter strafeAxis,
                            AxisGetter rotationAxis, boolean openLoop, boolean alwaysSlowMode) {
        m_swerveDrive = swerveDrive;
        addRequirements(swerveDrive);

        m_settings = settings;

        m_translationAxis = translationAxis;
        m_strafeAxis = strafeAxis;
        m_rotationAxis = rotationAxis;
        m_openLoop = openLoop;

        m_alwaysSlowMode = alwaysSlowMode;
    }

    private static double curveInput(double input) {
        return input * input * input;
    }

    /**
     * Applies deadband, clamps input, and curves input
     */
    private static double processLinearInput(double input) {
        // apply deadband
        if(Math.abs(input) < OperatorConstants.kJoystickDeadband) {
            // no need for further processing
            return 0;
        }

        // clamp input to range [-1, 1] and curve
        return curveInput(MathUtil.clamp(input, -1, 1));
    }

    @Override
    public void execute() {
        // xbox controller provides negative values when pushing forward (y axis), so negate
        // negate strafe (left/right x axis stick) as we want positive when pushing left (positive y on field)
        // negate rotation as we want positive value when when pushing left (CCW is postive)
        Translation2d translation = new Translation2d(-m_strafeAxis.getAxisValue(), -m_translationAxis.getAxisValue());
        double rAxis = -m_rotationAxis.getAxisValue();

        translation = new Translation2d(processLinearInput(translation.getNorm()), translation.getAngle());
        rAxis = processLinearInput(rAxis);

        double yAxis = translation.getY();
        double xAxis = translation.getX();
        
        // controller yAxis is forward movement, so is passed as x component of the translation
        // multiply [-1, 1] user input by velocities as these are desired chassis speeds
        double forward = yAxis * SwerveConstants.kMaxSpeed;
        double strafe = xAxis * SwerveConstants.kMaxSpeed;
        double rotation = rAxis * SwerveConstants.kMaxAngularVelocity;

        boolean slowMode = m_alwaysSlowMode || m_settings.isSlowMode();
        m_swerveDrive.drive(forward, strafe, rotation, m_settings.isFieldRelative(), slowMode, m_openLoop);
    }
}