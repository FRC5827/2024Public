package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

/**
 * Moves the robot straight for a specified distance
 * For small distances
*/
public final class MoveStraightCommand extends Command {

    private final SwerveDrive m_swerveDrive;

    private final double m_distance;

    private Transform2d m_transform;

    private boolean m_isFinished;

    public MoveStraightCommand(SwerveDrive swerveDrive, double distance) {
        m_swerveDrive = swerveDrive;
        m_distance = distance;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        // Creates a tranform for converting from field relative to robot relative based on the current position of the robot
        m_transform = new Pose2d().minus(m_swerveDrive.getPose());

        m_isFinished = false;
    }

    @Override
    public void execute() {
        // Converting the odometer to robot relative when the command is initialized
        Pose2d pose = m_swerveDrive.getPose().transformBy(m_transform);
        var delta = m_distance - pose.getX();

        if (delta * Math.signum(m_distance) > 0.01) {
            m_swerveDrive.drive(0.4 * Math.signum(delta), 0.0, 0.0, false, false, true);
        } else {
            m_isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }
}