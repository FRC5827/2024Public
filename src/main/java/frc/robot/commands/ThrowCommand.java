package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.ThrowerSubsystem;

public final class ThrowCommand extends Command {
    private final ThrowerSubsystem m_throwerSubsystem;
    private final HolderSubsystem m_holderSubsystem;

    private final boolean m_isAmp;
    private final Supplier<Boolean> m_readyToThrow;

    private ThrowCommand(HolderSubsystem holderSubsystem, ThrowerSubsystem throwerSubsystem, boolean isAmp, Supplier<Boolean> readyToThrow) {
        m_throwerSubsystem = throwerSubsystem;
        m_holderSubsystem = holderSubsystem;
        addRequirements(m_throwerSubsystem);
        addRequirements(m_holderSubsystem);

         m_isAmp = isAmp;
         m_readyToThrow = readyToThrow;
    }

    public static Command forSpeakerAutoFire(HolderSubsystem holderSubsystem, ThrowerSubsystem throwerSubsystem) {
        return create(holderSubsystem, throwerSubsystem, false, ShooterConstants.kShooterTimeout, () -> true);
    }

    public static Command forSpeakerTrigger(HolderSubsystem holderSubsystem, ThrowerSubsystem throwerSubsystem) {
        return create(holderSubsystem, throwerSubsystem, false, 0, () -> true);
    }

    public static Command forAmpTrigger(HolderSubsystem holderSubsystem, ThrowerSubsystem throwerSubsystem, ArmSubsystem armSubystem) {
        Command command = new ArmUpCommand(armSubystem);
        command = command.andThen(create(holderSubsystem, throwerSubsystem, true, 0, () -> true));
        command = command.finallyDo(interrupted -> armSubystem.moveDown());
        return command;
    }

    public static Command forAmpAutoFire(HolderSubsystem holderSubsystem, ThrowerSubsystem throwerSubsystem, ArmSubsystem armSubystem, SwerveDrive swerveDrive) {
        Command command = new ArmUpCommand(armSubystem);
        command = command.andThen(create(holderSubsystem, throwerSubsystem, true, ShooterConstants.kShooterTimeout * 2, () -> true));

        if (swerveDrive != null) {
            command = command.andThen(new MoveStraightCommand(swerveDrive, 0.1).withTimeout(2.5));
            command = command.andThen(new MoveStraightCommand(swerveDrive, -0.1).withTimeout(2));
        }
        
        command = command.finallyDo(interrupted -> armSubystem.moveDown());
        return command;
    }

    public static Command create(HolderSubsystem holderSubsystem, ThrowerSubsystem throwerSubsystem, boolean isAmp, double timeout, Supplier<Boolean> readyToThrow) {
        Command command = new ThrowCommand(holderSubsystem, throwerSubsystem, isAmp, readyToThrow);
        if (timeout > 0) {
            command = command.withTimeout(timeout);
        }

        return command;
    } 

    public static Command forWarmup(ThrowerSubsystem throwerSubsystem) {
        return new FunctionalCommand(() -> throwerSubsystem.start(false), () -> {}, interrupted -> throwerSubsystem.stop(), () -> false, throwerSubsystem);        
    }

    @Override
    public void initialize() {
        m_throwerSubsystem.start(m_isAmp);
    }

    @Override
    public void execute() {
        if (m_throwerSubsystem.isAtTargetSpeed()) {
            if (m_readyToThrow == null || m_readyToThrow.get()) {
                m_holderSubsystem.start(m_isAmp);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_throwerSubsystem.stop();
        m_holderSubsystem.stop();
    }
}