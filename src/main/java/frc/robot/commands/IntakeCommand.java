package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.ThrowerSubsystem;

public final class IntakeCommand extends Command {
    private final ThrowerSubsystem m_throwerSubsystem;
    private final HolderSubsystem m_holderSubsystem;
    private final CommandXboxController m_controller;

    public IntakeCommand(ThrowerSubsystem throwSubsystem, HolderSubsystem holderSubsystem) {
        this(throwSubsystem, holderSubsystem, null);
    }

    public IntakeCommand(ThrowerSubsystem throwerSubsystem, HolderSubsystem holderSubsystem, CommandXboxController controller) {
        m_throwerSubsystem = throwerSubsystem;
        m_holderSubsystem = holderSubsystem;
        m_controller = controller;
        addRequirements(m_throwerSubsystem);
        addRequirements(m_holderSubsystem);
    }

    @Override
    public void execute() {
        double powerAdjustment = 1;
        if (m_controller != null) {
            powerAdjustment = m_controller.getLeftTriggerAxis();
        }

        m_throwerSubsystem.intake(powerAdjustment);
        m_holderSubsystem.intake(powerAdjustment);
    }

    @Override
    public void end(boolean interrupted) {
        m_throwerSubsystem.stop();
        m_holderSubsystem.stop();
    }
}
