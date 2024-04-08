package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HooksConstants;

public class ClimberSubsystem extends SubsystemBase {
    private enum ClimberState {
        Zeroing, Stopped, Climbing, Raising
    }

    // declare motors and limit switches and gyro
    private final TalonFX m_hook1 = new TalonFX(HooksConstants.kMotorID[0]);
    private final TalonFX m_hook2 = new TalonFX(HooksConstants.kMotorID[1]);
    private final DigitalInput m_limitSwitch1 = new DigitalInput(HooksConstants.kLimitSwitchID[0]);
    private final DigitalInput m_limitSwitch2 = new DigitalInput(HooksConstants.kLimitSwitchID[1]);
    private final StatusSignal<Double> m_hook1Pos, m_hook2Pos;
    private final Supplier<Float> m_rollAxisSupplier;

    // declare voltageout and pid controllers for hooks
    // voltageout requests voltage from motors
    // pid controllers are used to do pid loops
    private final VoltageOut m_voltageOut = new VoltageOut(0);
    private final PIDController m_movementpid = new PIDController(0.5, 0, 0);
    private final PIDController m_balancepid = new PIDController(0.25, 0, 0);

    private ClimberState state;
    private double m_gyroOffset;

    // constructor
    // set gyro
    public ClimberSubsystem(Supplier<Float> rollAxisSupplier){
        m_rollAxisSupplier = rollAxisSupplier;
        m_hook1.setPosition(0);
        m_hook2.setPosition(0);
        m_movementpid.setTolerance(0.5);
        m_balancepid.setSetpoint(0);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Prevents hooks from going past 4 feet limit
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HooksConstants.kUpperLimit;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        config.MotorOutput.Inverted = HooksConstants.kInverted[0] ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        m_hook1.getConfigurator().apply(config);

        config.MotorOutput.Inverted = HooksConstants.kInverted[1] ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        m_hook2.getConfigurator().apply(config);

        m_hook1Pos = m_hook1.getPosition();
        m_hook2Pos = m_hook2.getPosition();

        state = ClimberState.Zeroing;
        m_gyroOffset = 0;
    }

    public void setGyroOffset() {
        m_gyroOffset = -m_rollAxisSupplier.get();
    }

    // function to raise hooks
    public void raiseHook(){
        state = ClimberState.Raising;
        m_movementpid.setSetpoint(HooksConstants.kUpperLimit);
    }
    // function to lower hooks
    public void lowerHook(){
        state = ClimberState.Climbing;
        m_movementpid.setSetpoint(0);
    }
    // function to stop hooks
    public void stopHook(){
        state = ClimberState.Stopped;
    }
    // function to zero hooks
    public void zeroHook() {
        if(state == ClimberState.Zeroing) {
            state = ClimberState.Stopped;
        } else {
            state = ClimberState.Zeroing;
        }
    }

    private void setVoltage(TalonFX motor, double voltage, double position) {
        if(position <= 0 && voltage < 0) {
            motor.setControl(m_voltageOut.withOutput(0));
            return;
        }
        motor.setControl(m_voltageOut.withOutput(voltage));
    }
    // periodic runs every 20 ms
    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(m_hook1Pos, m_hook2Pos);
        if(state == ClimberState.Zeroing) {
            if(m_limitSwitch1.get()) {
                m_hook1.setControl(m_voltageOut.withOutput(0));
            } else {
                m_hook1.setControl(m_voltageOut.withOutput(-HooksConstants.kZeroingSpeed));
            }
            if(m_limitSwitch2.get()) {
                m_hook2.setControl(m_voltageOut.withOutput(0));
            } else {
                m_hook2.setControl(m_voltageOut.withOutput(-HooksConstants.kZeroingSpeed));
            }
            if(m_limitSwitch1.get() && m_limitSwitch2.get()){
                state = ClimberState.Stopped;
                m_movementpid.setSetpoint(0);
                m_hook1.setPosition(0);
                m_hook2.setPosition(0);
            }
        } else if(state == ClimberState.Stopped) {
            m_hook1.set(0);
            m_hook2.set(0);
        } else {
            double hook1Pos = m_hook1Pos.getValue();
            double hook2Pos = m_hook2Pos.getValue();

            // In case motor encoders drift
            if(m_limitSwitch1.get() && Math.abs(hook1Pos) >= 0.1) {
                m_hook1.setPosition(0);
            }
            if(m_limitSwitch2.get() && Math.abs(hook2Pos) >= 0.1) {
                m_hook2.setPosition(0);
            }

            // Get the voltage offset from PID loop
            double balancingOffset;
            if(state == ClimberState.Climbing) {
                // Level the hooks relative to the ground to balance
                balancingOffset = m_balancepid.calculate(m_rollAxisSupplier.get() + m_gyroOffset);
            } else {
                // Don't change balance if raising hooks
                balancingOffset = 0;
            }

            balancingOffset = MathUtil.clamp(balancingOffset, -HooksConstants.kBalancingSpeed, HooksConstants.kBalancingSpeed);
            double hook1BalancingOffset = 0, hook2BalancingOffset = 0;
            // Make sure the balancing offset doesn't cause the position of the lowest hook to go lower
            // Prevents trying to go below limit switch
            if(hook1Pos <= hook2Pos) {
                hook2BalancingOffset = balancingOffset;
            } else {
                hook1BalancingOffset = -balancingOffset;
            }

            double movementOutput = MathUtil.clamp(m_movementpid.calculate(Math.min(hook1Pos, hook2Pos)), -HooksConstants.kMotorSpeed, HooksConstants.kMotorSpeed);

            setVoltage(m_hook1, movementOutput + hook1BalancingOffset, hook1Pos);
            setVoltage(m_hook2, movementOutput + hook2BalancingOffset, hook2Pos);

            SmartDashboard.putNumber("Climber/movementpidOutput", movementOutput);
            SmartDashboard.putNumber("Climber/hook1balancingoffset", hook1BalancingOffset);
            SmartDashboard.putNumber("Climber/hook2balancingoffset", hook2BalancingOffset);
        }
        SmartDashboard.putBoolean("Climber/Limit Switch 1", m_limitSwitch1.get());
        SmartDashboard.putBoolean("Climber/Limit Switch 2", m_limitSwitch2.get());
        SmartDashboard.putNumber("Climber/Climber 1 Position", m_hook1Pos.getValueAsDouble());
        SmartDashboard.putNumber("Climber/Climber 2 Position", m_hook2Pos.getValueAsDouble());
        SmartDashboard.putNumber("Climber/RawTilt", m_rollAxisSupplier.get());
    }

    public boolean isHooksUp() {
        return m_hook1Pos.getValue() >= HooksConstants.kUpperLimit-5 && m_hook2Pos.getValue() >= HooksConstants.kUpperLimit-5;
    }
}
