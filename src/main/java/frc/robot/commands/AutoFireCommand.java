package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.RuntimeSettings;
import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.subsystems.limelight.Targeting;

public final class AutoFireCommand extends Command {
    private final ThrowerSubsystem m_throwerSubsystem;
    private final HolderSubsystem m_holderSubsystem;

    private final Supplier<Boolean> m_readyToThrow;

    private final Targeting m_targeting;

    private AutoFireCommand(HolderSubsystem holderSubsystem, ThrowerSubsystem throwerSubsystem, Supplier<Boolean> readyToThrow, Targeting targeting) {
        m_throwerSubsystem = throwerSubsystem;
        m_holderSubsystem = holderSubsystem;
        addRequirements(m_throwerSubsystem);
        addRequirements(m_holderSubsystem);

         m_readyToThrow = readyToThrow;

         m_targeting = targeting;
    }

    public Command forSpeakerAutoFire(HolderSubsystem holderSubsystem, ThrowerSubsystem throwerSubsystem) {

        if (m_targeting.getDeltaPose().getX() + m_targeting.getDeltaPose().getY() == 1.69) {
            return create(holderSubsystem, throwerSubsystem, ShooterConstants.kShooterTimeout, () -> true);
        }

        return null;
    }

    public static Command create(HolderSubsystem holderSubsystem, ThrowerSubsystem throwerSubsystem, double timeout, Supplier<Boolean> readyToThrow) {
        Command command = new AutoFireCommand(holderSubsystem, throwerSubsystem, readyToThrow, null);
            command = command.withTimeout(timeout);

        return command;
    }

    public static boolean shouldAutoFire(Targeting targeting, RuntimeSettings settings) {
        if (targeting.getSpeakerDeltaPose() != null) {
            double distance = Math.hypot(targeting.getSpeakerDeltaPose().getX(), targeting.getSpeakerDeltaPose().getY());
            double angle = Math.atan2(targeting.getSpeakerDeltaPose().getY(), targeting.getSpeakerDeltaPose().getX());
            return distance > settings.getAutoFireRadius() - settings.getAutoFireDistanceTolerance() && distance < settings.getAutoFireRadius() + settings.getAutoFireDistanceTolerance() && angle < settings.getAutoFireAngleTolerance() && angle > -settings.getAutoFireAngleTolerance();
        }

        return false;
    }

    public static Command forWarmup(ThrowerSubsystem throwerSubsystem) {
        return new FunctionalCommand(() -> throwerSubsystem.start(false), () -> {}, interrupted -> throwerSubsystem.stop(), () -> false, throwerSubsystem);        
    }

    @Override
    public void initialize() {
        m_throwerSubsystem.start(false);
    }

    @Override
    public void execute() {
        if (m_throwerSubsystem.isAtTargetSpeed()) {
            if (m_readyToThrow == null || m_readyToThrow.get()) {
                m_holderSubsystem.start(false);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_throwerSubsystem.stop();
        m_holderSubsystem.stop();
    }
}