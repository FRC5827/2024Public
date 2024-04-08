package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.limelight.Targeting;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class AlignToTargetCommand extends Command {
    private final Targeting m_targeting;
    private final SwerveDrive m_swerveDrive;
    private final boolean m_doNotDrive; // for testing ony

    private final ProfiledPIDController m_xController = new ProfiledPIDController(1.5, 0, 0,
        new TrapezoidProfile.Constraints(3, 1.5));
    private final ProfiledPIDController m_yController = new ProfiledPIDController(1.5, 0, 0,
        new TrapezoidProfile.Constraints(3, 1.5));
    private final ProfiledPIDController m_rController = new ProfiledPIDController(1.5, 0, 0,
        new TrapezoidProfile.Constraints(6.28, 3.14));

    public AlignToTargetCommand(Targeting targeting, SwerveDrive swerveDrive) {
        this(targeting, swerveDrive, false);
    }

    public AlignToTargetCommand(Targeting targeting, SwerveDrive swerveDrive, boolean doNotDrive) {
        m_targeting = targeting;
        m_swerveDrive = swerveDrive;
        m_doNotDrive = doNotDrive;
        if (!doNotDrive) {
            addRequirements(m_swerveDrive);
        }

        m_xController.setTolerance(0.1);
        m_yController.setTolerance(0.1);
        m_rController.setTolerance(Units.degreesToRadians(5));        
        m_xController.disableContinuousInput();
        m_yController.disableContinuousInput();
        m_xController.setIntegratorRange(-0.2, 0.2);
        m_yController.setIntegratorRange(-0.2, 0.2);
        m_rController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        var delta = m_targeting.getDeltaPose();
        double dx = m_xController.calculate(-delta.getX());
        double dy = m_yController.calculate(-delta.getY());
        double dr = m_rController.calculate(-delta.getRotation().getRadians());

        if (!m_doNotDrive) {
            m_swerveDrive.drive(dx, dy, dr, false, false, true);
        }
    }

    public boolean isFinished() {
        var isAligned = m_xController.atSetpoint() && m_yController.atSetpoint() && m_rController.atSetpoint();
        if (isAligned) {
            m_targeting.setAligned(true);
        }
        return isAligned;
    }
}
