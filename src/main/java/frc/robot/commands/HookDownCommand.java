package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;

public final class HookDownCommand extends Command {
    private final ClimberSubsystem m_climberSubsystem;

    public HookDownCommand(ClimberSubsystem climberSubsystem){
        m_climberSubsystem = climberSubsystem;
        addRequirements(m_climberSubsystem);
    }

    @Override
    public void initialize(){
        m_climberSubsystem.lowerHook();
    }

    @Override
    public void end(boolean interrupted){
        m_climberSubsystem.stopHook();
    }
}
