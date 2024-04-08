package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;

public final class HookUpCommand extends Command {
    private final ClimberSubsystem m_climberSubsystem;
    private final boolean m_endWhenRaised;

    public HookUpCommand(ClimberSubsystem climberSubsystem){
        this(climberSubsystem, false);
    }

    public HookUpCommand(ClimberSubsystem climberSubsystem, boolean endWhenRaised){
        m_climberSubsystem = climberSubsystem;
        addRequirements(m_climberSubsystem);
        m_endWhenRaised = endWhenRaised;
    }

    @Override
    public void initialize(){
        m_climberSubsystem.raiseHook();
    }

    @Override
    public boolean isFinished() {
        if (!m_endWhenRaised) {
            return false;
        }

        return m_climberSubsystem.isHooksUp();
    }

    @Override
    public void end(boolean interrupted){
        m_climberSubsystem.stopHook();
    }
}
