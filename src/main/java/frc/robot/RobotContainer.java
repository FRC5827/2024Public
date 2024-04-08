// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlignToTargetCommand;
import frc.robot.commands.ArmUpHalfwayCommand;
import frc.robot.commands.AutoFireCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveStraightCommand;
import frc.robot.commands.PassingCommand;
import frc.robot.commands.ThrowCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.HookUpCommand;
import frc.robot.commands.HookDownCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.Constants.RioConstants;
import frc.robot.subsystems.limelight.LimeLight;
import frc.robot.subsystems.limelight.TargetType;
import frc.robot.subsystems.limelight.Targeting;
import frc.robot.subsystems.RuntimeSettings;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.subsystems.SwerveDrive;

import java.util.EnumMap;
import java.util.function.BooleanSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandXboxController m_driverController;

    private final RuntimeSettings m_settings;
    private final AHRS m_gyro;
    private final LimeLight m_limelight;
    private final Targeting m_targeting;

    private final SwerveDrive m_swerveDrive;
    private final ThrowerSubsystem m_throwerSubsystem;
    private final HolderSubsystem m_holderSubsystem;
    private final ClimberSubsystem m_hooks;
    private final ArmSubsystem m_ampScorerSubsystem;

    // Sendable chooser for auto command
    private final SendableChooser<Command> m_autoChooser;
    private final SendableChooser<Command> m_autoDelay;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Initialization subsystems based o nwhether this is Swerve or Competition bot.
        m_settings = new RuntimeSettings();
        m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

        // Conditional subsystems
        if (RioConstants.hasSwerveDrive() || RioConstants.hasClimber()) {
            m_gyro = new AHRS(SPI.Port.kMXP);
        } else {
            m_gyro = null;
        }

        if (RioConstants.hasClimber()) {
            m_hooks = new ClimberSubsystem(m_gyro::getRoll);
        } else {
            m_hooks = null;
        }

        if (RioConstants.hasSwerveDrive()) {
            m_swerveDrive = new SwerveDrive(m_gyro);
            m_limelight = new LimeLight();
            m_targeting = new Targeting(m_limelight, m_driverController, m_swerveDrive::getPose);
        } else {
            m_swerveDrive = null;
            m_limelight = null;
            m_targeting = null;
        }

        if (RioConstants.hasShooter()) {
            m_throwerSubsystem = new ThrowerSubsystem();
            m_holderSubsystem = new HolderSubsystem();
        } else {
            m_throwerSubsystem = null;
            m_holderSubsystem = null;
        }

        if(RioConstants.hasAmpArm()) {
            m_ampScorerSubsystem = new ArmSubsystem();
        } else {
            m_ampScorerSubsystem = null;
        }

        // Register the auton NamedCommands
        registerNamedCommands();

        // Configure the button bindings
        configureButtonBindings();

        if (RioConstants.hasSwerveDrive()) {
            m_swerveDrive.setDefaultCommand(new TeleopSwerve(
                m_swerveDrive,
                m_settings,
                m_driverController::getLeftY,
                m_driverController::getLeftX,
                m_driverController::getRightX,
                true,
                false));

            m_autoChooser = AutoBuilder.buildAutoChooser();
            m_autoDelay = new SendableChooser<Command>();
            m_autoDelay.setDefaultOption("Default", new WaitCommand(0));
            for(int i = 1; i < 10; i++){
                m_autoDelay.addOption("Wait " + i + " Seconds", new WaitCommand(i));
            }

            Shuffleboard.getTab("Autonomous").add("Autonomous Sequence", m_autoChooser).withPosition(0, 0)
                                                .withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
            Shuffleboard.getTab("Autonomous").add("Delay", m_autoDelay).withPosition(2, 0)
                                                .withSize(2,1).withWidget(BuiltInWidgets.kComboBoxChooser);
        } else {
            m_autoChooser = null;
            m_autoDelay = null;
        }
    }

    private Command getAmpAutoAlignCommand() {
        Command warmup = new PrintCommand("No warmup");
        Command action = new PrintCommand("no action");

        if (RioConstants.hasAmpArm() && RioConstants.hasShooter()) {
            warmup = new ArmUpHalfwayCommand(m_ampScorerSubsystem);
            action = ThrowCommand.forAmpAutoFire(m_holderSubsystem, m_throwerSubsystem, m_ampScorerSubsystem, m_swerveDrive);
        }

        return getWarmupAndActionSequentialCommand(warmup, action);
    }

    private Command getSpeakerAutoAlignCommand() {
        Command warmup = new PrintCommand("No warmup");
        Command action = new PrintCommand("no action");

        if (RioConstants.hasShooter()) {
            action = ThrowCommand.forSpeakerAutoFire(m_holderSubsystem, m_throwerSubsystem);
        }

        return getWarmupAndActionSequentialCommand(warmup, action);
    }

    private Command getStageAutoAlignCommand() {
        Command warmup = new PrintCommand("No warmup");
        Command action = new PrintCommand("no action");

        if (RioConstants.hasClimber()) {
            warmup = new HookUpCommand(m_hooks);
            action = new SequentialCommandGroup(
                new HookUpCommand(m_hooks, true),
                new MoveStraightCommand(m_swerveDrive, -0.1).withTimeout(2),
                new HookDownCommand(m_hooks));
        }

        return getWarmupAndActionSequentialCommand(warmup, action);
    }

    private Command getSourceAutoAlignCommand() {
        Command warmup = new PrintCommand("No warmup");
        Command action = new PrintCommand("no action");

        if (RioConstants.hasShooter()) {
            warmup = new IntakeCommand(m_throwerSubsystem, m_holderSubsystem);
            action = new ParallelCommandGroup(new IntakeCommand(m_throwerSubsystem, m_holderSubsystem), new MoveStraightCommand(m_swerveDrive, 0.06));
        }

        return getWarmupAndActionSequentialCommand(warmup, action);
    }

    private Command getNoneAutoAlignCommand() {
        Command warmup = new PrintCommand("No warmup");
        Command action = new PrintCommand("no action");
        return getWarmupAndActionSequentialCommand(warmup, action);
    }

    private Command getWarmupAndActionSequentialCommand(Command warmup, Command action)
    {
        return getWarmupAndActionSequentialCommand(warmup, action, null);
    }

    private Command getWarmupAndActionSequentialCommand(Command warmup, Command action, BooleanSupplier readyForAction)
    {
        if (readyForAction == null) {
            return new SequentialCommandGroup(
                // Wait until we can get a target lock
                new WaitUntilCommand(m_targeting::isTargetLocked),            
                new ParallelDeadlineGroup(new AlignToTargetCommand(m_targeting, m_swerveDrive), warmup),
                action);
        }

        return new SequentialCommandGroup(
            // Wait until we can get a target lock
            new WaitUntilCommand(m_targeting::isTargetLocked),            
            new ParallelDeadlineGroup(
                new ParallelRaceGroup(
                    new AlignToTargetCommand(m_targeting, m_swerveDrive),
                    new WaitUntilCommand(readyForAction)
                ),
                warmup),
            action);
    }

    private Command getAutoAlignCommand() {
        var commands = new EnumMap<TargetType, Command>(TargetType.class);
        commands.put(TargetType.Amp, getAmpAutoAlignCommand());
        commands.put(TargetType.Speaker, getSpeakerAutoAlignCommand());
        commands.put(TargetType.Stage, getStageAutoAlignCommand());
        commands.put(TargetType.Source, getSourceAutoAlignCommand());
        commands.put(TargetType.None, getNoneAutoAlignCommand());
        
        return new ParallelDeadlineGroup(
            // The deadline group is used to run the main logic while locked. The ParallelDeadlineGroup will not end
            // until the main SequentialCommandGroup ends.
            new SelectCommand<>(commands, () -> m_targeting.getLockedTargetType()),

            // Non-deadline command runs in parallel to lock the target and keep it locked.
            new FunctionalCommand(() ->{}, () -> m_targeting.lockTarget(), interrupted -> m_targeting.releaseTarget(), () -> false)
        );
    }
    
    private void registerNamedCommands() {
        if (RioConstants.hasSwerveDrive() && RioConstants.hasAmpArm() && RioConstants.hasShooter()){
            NamedCommands.registerCommand("AutoAlignAndScore", getAutoAlignCommand());
            NamedCommands.registerCommand("AutoFire", ThrowCommand.forSpeakerAutoFire(m_holderSubsystem, m_throwerSubsystem));
        }
    }

    private void configureButtonBindings() {
        // Add button bindings here
        if (RioConstants.hasShooter()) {
            // Creates a trigger that is more sensitive than the default
            Trigger sensitiveLeftTrigger = new Trigger(() -> m_driverController.getLeftTriggerAxis() > Constants.ShooterConstants.kMinAxisValue);
            sensitiveLeftTrigger.whileTrue(new IntakeCommand(m_throwerSubsystem, m_holderSubsystem, m_driverController));
            m_driverController.rightTrigger().whileTrue(ThrowCommand.forSpeakerTrigger(m_holderSubsystem, m_throwerSubsystem));

            //m_driverController.povRight().whileTrue(new PassingCommand(m_holderSubsystem, m_throwerSubsystem, null));
        }

        if (RioConstants.hasShooter() && RioConstants.hasAmpArm()) {
            m_driverController.rightBumper().whileTrue(ThrowCommand.forAmpTrigger(m_holderSubsystem, m_throwerSubsystem, m_ampScorerSubsystem));
        }

        if (RioConstants.hasSwerveDrive()) {
            m_driverController.rightStick().whileTrue(new TeleopSwerve(
                m_swerveDrive,
                m_settings,
                m_driverController::getLeftY,
                m_driverController::getLeftX,
                m_driverController::getRightX,
                true,
                true));
        
            m_driverController.back().onTrue(new InstantCommand(() -> {
                m_settings.setSlowMode(!m_settings.isSlowMode());
            }));

            m_driverController.start().onTrue(new InstantCommand(() -> {
                m_settings.setFieldRelative(!m_settings.isFieldRelative());
            }));
        }

        if (RioConstants.hasSwerveDrive()) {
            m_driverController
                .a()
                .and(() -> m_targeting.isTargetLocked() || m_targeting.hasCandidateTarget())
                .whileTrue(getAutoAlignCommand());
            m_driverController.povUp().onTrue(new InstantCommand(() -> m_swerveDrive.zeroHeading()));

            m_driverController
                .leftBumper()
                .and(() -> m_targeting.isTargetLocked() || m_targeting.hasCandidateTarget()).and(() -> AutoFireCommand.shouldAutoFire(m_targeting, m_settings))
                .whileTrue(ThrowCommand.forSpeakerAutoFire(m_holderSubsystem, m_throwerSubsystem));
        }

        if (RioConstants.hasClimber()) {
            m_driverController.y().whileTrue(new HookUpCommand(m_hooks));
            m_driverController.x().whileTrue(new HookDownCommand(m_hooks));
            m_driverController.povLeft().onTrue(new InstantCommand(() -> m_hooks.zeroHook()));
            m_driverController.povRight().onTrue(new InstantCommand(() -> m_hooks.setGyroOffset()));
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(){
       return new ProxyCommand(() -> m_autoDelay.getSelected()).andThen(new ProxyCommand(() -> m_autoChooser.getSelected()));
    }
}