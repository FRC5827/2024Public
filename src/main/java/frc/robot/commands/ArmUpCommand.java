package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public final class ArmUpCommand extends Command {
    private final ArmSubsystem m_armSubsystem;

    public ArmUpCommand(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        m_armSubsystem.moveUp();
    }

    @Override
    public boolean isFinished() {
        return m_armSubsystem.isFinished();
    }
}
