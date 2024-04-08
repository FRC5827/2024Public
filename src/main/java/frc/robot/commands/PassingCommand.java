package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.ThrowerSubsystem;

public final class PassingCommand extends Command {
    private final ThrowerSubsystem m_throwerSubsystem;
    private final HolderSubsystem m_holderSubsystem;

    private final Supplier<Boolean> m_readyToThrow;

    public PassingCommand(HolderSubsystem holderSubsystem, ThrowerSubsystem throwerSubsystem, Supplier<Boolean> readyToThrow) {
        m_throwerSubsystem = throwerSubsystem;
        m_holderSubsystem = holderSubsystem;
        addRequirements(m_throwerSubsystem);
        addRequirements(m_holderSubsystem);

        m_readyToThrow = readyToThrow;
    }

    @Override
    public void initialize() {
        m_throwerSubsystem.startSpin();;
    }

    @Override
    public void execute() {
        if (m_throwerSubsystem.isAtTargetSpeed()) {
            if (m_readyToThrow == null || m_readyToThrow.get()) {
                m_holderSubsystem.startSpin();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_throwerSubsystem.stop();
        m_holderSubsystem.stop();
    }
}