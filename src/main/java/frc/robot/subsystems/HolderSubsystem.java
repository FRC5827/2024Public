package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Subsystem that holds the note in shooter.
 */
public final class HolderSubsystem extends SubsystemBase {
    private final TalonFX m_holderLeader = new TalonFX(ShooterConstants.kHolderMotorIDs[0]);
    private final TalonFX m_holderFollower = new TalonFX(ShooterConstants.kHolderMotorIDs[1]);
    
    private final VelocityVoltage m_targetVelocity = new VelocityVoltage(0);

    public HolderSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.Slot0 = ShooterConstants.kHolderConfigs;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_holderLeader.getConfigurator().apply(configs);
        m_holderFollower.getConfigurator().apply(configs);
    }

    public void stop() {
        m_targetVelocity.Velocity = 0;
        m_holderLeader.set(0);
        m_holderFollower.set(0);
    }

    public void intake() {
        intake(1);
    }

    public void intake(double multiplier) {
        multiplier = Math.max(0, multiplier);
        multiplier = Math.min(multiplier, 1);
        m_targetVelocity.Velocity = ShooterConstants.kMaxIntakePower * multiplier;
        m_holderLeader.setControl(m_targetVelocity);
        m_holderFollower.setControl(new VelocityVoltage(-m_targetVelocity.Velocity));
    }

    public void start(boolean isAmp) {
        m_targetVelocity.Velocity = isAmp ? ShooterConstants.kAmpShooterPower : ShooterConstants.kSpeakerShooterPower;
        m_holderLeader.setControl(m_targetVelocity);
        m_holderFollower.setControl(new VelocityVoltage(-m_targetVelocity.Velocity));
    }

    public void startSpin() {
        m_targetVelocity.Velocity = ShooterConstants.kSpeakerShooterPower;
        m_holderLeader.setControl(m_targetVelocity);
        m_holderFollower.setControl(new VelocityVoltage(-m_targetVelocity.Velocity * 0.8));
    }
}