package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Subsystem that throws the note from the shooter.
 */
public final class ThrowerSubsystem extends SubsystemBase {
    private final TalonFX m_shooterLeader = new TalonFX(ShooterConstants.kShooterMotorIDs[0]);
    private final TalonFX m_shooterFollower = new TalonFX(ShooterConstants.kShooterMotorIDs[1]);
    
    private final VelocityVoltage m_targetVelocity = new VelocityVoltage(0);

    private final StatusSignal<Double> m_velocitySignal;

    public ThrowerSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.Slot0 = ShooterConstants.kShooterConfigs;

        m_shooterLeader.getConfigurator().apply(configs);
        m_shooterFollower.getConfigurator().apply(configs);
        m_velocitySignal = m_shooterLeader.getVelocity();
    }

    public void stop() {
        m_targetVelocity.Velocity = 0;
        m_shooterLeader.set(0);
        m_shooterFollower.set(0);
    }

    public void intake() {
        intake(1);
    }

    public void intake(double multiplier) {
        multiplier = Math.max(0, multiplier);
        multiplier = Math.min(multiplier, 1);
        m_targetVelocity.Velocity = ShooterConstants.kMaxIntakePower * multiplier;
        m_shooterLeader.setControl(m_targetVelocity);
        m_shooterFollower.setControl(new VelocityVoltage(-m_targetVelocity.Velocity));
    }

    public void start(boolean isAmp) {
        m_targetVelocity.Velocity = isAmp ? ShooterConstants.kAmpShooterPower : ShooterConstants.kSpeakerShooterPower;
        m_shooterLeader.setControl(m_targetVelocity);
        m_shooterFollower.setControl(new VelocityVoltage(-m_targetVelocity.Velocity));
    }

    public void startSpin() {
        m_targetVelocity.Velocity = ShooterConstants.kSpeakerPassShooterPower;
        m_shooterLeader.setControl(m_targetVelocity);
        m_shooterFollower.setControl(new VelocityVoltage(-m_targetVelocity.Velocity * ShooterConstants.kSpeakerPassShooterOffset));
    }

    public boolean isAtTargetSpeed() {
        return (m_velocitySignal.refresh().getValue() >= m_targetVelocity.Velocity - ShooterConstants.kVelocityTolerance);
    }
}
