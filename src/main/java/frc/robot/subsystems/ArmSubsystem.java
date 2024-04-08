package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;

public final class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax m_motorController = new CANSparkMax(AmpConstants.kMotorID, MotorType.kBrushless);
    private final ProfiledPIDController m_PIDController = new ProfiledPIDController(AmpConstants.kP, 
                AmpConstants.kI, AmpConstants.kD, new TrapezoidProfile.Constraints(AmpConstants.kMaxVel, AmpConstants.kMaxAcc));

    private final CANcoder m_CANCoder;
    private final StatusSignal<Double> m_positionSignal;
    
    public ArmSubsystem() {
        m_CANCoder = new CANcoder(AmpConstants.kEncoderID);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = AmpConstants.kOffset;
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        m_CANCoder.getConfigurator().apply(config);
        m_positionSignal = m_CANCoder.getPosition();
        m_PIDController.setTolerance(AmpConstants.kTolerance);
        moveDown();
    }

    public void moveUp(){
        m_PIDController.setGoal(AmpConstants.kUpPIDSetpoint);
    }

    public void moveUpHalf(){
        m_PIDController.setGoal(AmpConstants.kUpHalfPidSetpoint);
    }

    public void moveDown() {
        m_PIDController.setGoal(AmpConstants.kDownPIDSetpoint);
    }

    public boolean isFinished() {
        return m_PIDController.atGoal();
    }

    public void stop() {
        // should be refreshed in periodic
        m_PIDController.setGoal(m_positionSignal.getValue());
    }

    @Override
    public void periodic() {
        double position = m_positionSignal.refresh().getValue();
        // voltage to hold it in place
        double kGValue = Math.cos(position * 2 * Math.PI) * AmpConstants.kG;
        double pidOutput = -m_PIDController.calculate(position);
        // add feedforward g value
        m_motorController.setVoltage(pidOutput + kGValue);
        SmartDashboard.putNumber("amp/pidOutput", pidOutput);
        SmartDashboard.putNumber("amp/kGValue", kGValue);
        SmartDashboard.putNumber("amp/position", position);
    }
}
