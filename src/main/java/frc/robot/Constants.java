// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class RioConstants {
        /** The serial numbers of the roboRIOs used for test bots */
        private static final String kSwerveBotRoboRIOSerialNum = "030588d2";
        private static final String kBoardBotRoboRIOSerialNum = "031ad12e";

        /** Returns true if the current roboRIO is the swervebot */
        public static boolean isSwerveBot() {
            String serialNo = System.getenv("serialnum");
            return kSwerveBotRoboRIOSerialNum.equalsIgnoreCase(serialNo);
        }

        /** Returns true if the current roboRIO is the board bot */
        public static boolean isBoardBot() {
            String serialNo = System.getenv("serialnum");
            return kBoardBotRoboRIOSerialNum.equalsIgnoreCase(serialNo);
        }

        /** Any other roboRIO is considered the racing (production) robot.
            This facilitates a fast switch of roboRIO hardware if necessary without code changes.
        */
        public static boolean isRacingBot() {
            return !isSwerveBot() && !isBoardBot();
        }

        /** Returns true if this is bot has a swerve drive */
        public static boolean hasSwerveDrive() {
            return isRacingBot() || isSwerveBot();
        }

        /** Returns true if this is bot has a shooter subsystem */
        public static boolean hasShooter() {
            return isRacingBot() || isBoardBot();
        }

        public static boolean hasAmpArm() {
            return isRacingBot();
        }

        /** Returns true if this is bot has a climber subsystem */
        public static boolean hasClimber() {
            return isRacingBot();
        }
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;

        public static final double kJoystickDeadband = 0.08;

        // slow mode default to true and field relative default to false for testing
        public static final boolean kDefaultSlowMode = false;
        public static final boolean kDefaultFieldRelative = true;
    }

    public static class HooksConstants{
        public static final int[] kMotorID = {4, 5};
        public static final boolean[] kInverted = {false, true};
        public static final double kMotorSpeed = 10;
        public static final double kBalancingSpeed = 2;
        public static final double kZeroingSpeed = 1.5;
        public static final int[] kLimitSwitchID = {6, 8};
        public static final double kUpperLimit = 125;
    }

    public static class AutoConstants {
        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(1.0, 0, 0),
            new PIDConstants(1.0, 0, 0),
            4.5,
            0.4,
            new ReplanningConfig()
        );

        public static boolean shouldFlipPath() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        }
    }

    public static class SwerveConstants {
        // always ensure gyro is counter-clock-wise positive
        public static final boolean kInvertGyro = true;

        public static final double kSlowModeFactor = 1.0/3.0;

        // drivetrain constants
        public static final double kTrackWidth = RioConstants.isSwerveBot() ? Units.inchesToMeters(22.5) : Units.inchesToMeters(20.5);
        public static final double kWheelBase = RioConstants.isSwerveBot() ? Units.inchesToMeters(22.5) : Units.inchesToMeters(20.5);

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        public static final Slot0Configs steerGains = RioConstants.isSwerveBot() ?
            // SwerveBot constants
            new Slot0Configs().withKP(18.736).withKI(0).withKD(0.512)
            .withKS(0.133).withKV(1.437).withKA(0.031) :
            // Competition robot constants
            new Slot0Configs().withKP(100).withKI(0).withKD(0.05)
            .withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        public static final Slot0Configs driveGains = RioConstants.isSwerveBot() ?
            // SwerveBot constants
            new Slot0Configs().withKP(0.059).withKI(0).withKD(0)
            .withKS(0.038).withKV(0.343).withKA(0.019) :
            // default constants, will be tuned for competition robot
            new Slot0Configs().withKP(0.5).withKI(0).withKD(0)
            .withKS(0).withKV(0).withKA(0);

        public static final String kCanBus = "rio";

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        public static final double kSlipCurrentA = 400.0;

        // Theoretical free speed (m/s) at 12v applied output
        public static final double kMaxSpeed = 4.5;
        public static final double kMaxAngularVelocity = 1.5 * (2 * Math.PI); // 1.5 rotations/s (radians)

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns
        public static final double kCoupleRatio = 3;

        public static final double kDriveGearRatio = 6.86;
        public static final double kSteerGearRatio = 12.8;
        public static final double kWheelRadiusInches = 2.0;

        public static final boolean kSteerMotorReversed = false;
        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = RioConstants.isSwerveBot() ? false : true;

        // These are only used for simulation
        public static final double kSteerInertia = 0.00001;
        public static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        public static final double kSteerFrictionVoltage = 0.25;
        public static final double kDriveFrictionVoltage = 0.25;

        public static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withWheelRadius(kWheelRadiusInches)
                .withSlipCurrent(kSlipCurrentA)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withSpeedAt12VoltsMps(kMaxSpeed)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
                .withCouplingGearRatio(kCoupleRatio)
                .withSteerMotorInverted(kSteerMotorReversed);

        // Front Left
        public static final class FrontLeft {
            public static final int kDriveMotorId = 10;
            public static final int kSteerMotorId = 11;
            public static final int kEncoderId = 20;

            public static final double kXPos = kWheelBase / 2.0;
            public static final double kYPos = kTrackWidth / 2.0;

            public static final double kSwerveBotEncoderOffset = 0.043457;
            public static final double kRacingBotEncoderOffset = -0.225098;

            public static double getEncoderOffset() {
                return RioConstants.isSwerveBot() ? kSwerveBotEncoderOffset : kRacingBotEncoderOffset;
            }

            public static final SwerveModuleConstants ModuleConstants = ConstantCreator.createModuleConstants(
                kSteerMotorId, kDriveMotorId, kEncoderId, getEncoderOffset(), kXPos, kYPos, kInvertLeftSide);
        }

        // Front Right
        public static final class FrontRight {
            public static final int kDriveMotorId = 12;
            public static final int kSteerMotorId = 13;
            public static final int kEncoderId = 21;

            public static final double kSwerveBotEncoderOffset = -0.215088;
            public static final double kRacingBotEncoderOffset = -0.208740;

            public static final double kXPos = kWheelBase / 2.0;
            public static final double kYPos = -kTrackWidth / 2.0;

            public static double getEncoderOffset() {
                return RioConstants.isSwerveBot() ? kSwerveBotEncoderOffset : kRacingBotEncoderOffset;
            }

            public static final SwerveModuleConstants ModuleConstants = ConstantCreator.createModuleConstants(
                kSteerMotorId, kDriveMotorId, kEncoderId, getEncoderOffset(), kXPos, kYPos, kInvertRightSide);
        }

        // Back Left
        public static final class BackLeft {
            public static final int kDriveMotorId = 14;
            public static final int kSteerMotorId = 15;
            public static final int kEncoderId = 22;

            public static final double kXPos = -kWheelBase / 2.0;
            public static final double kYPos = kTrackWidth / 2.0;

            public static final double kSwerveBotEncoderOffset = -0.386963;
            public static final double kRacingBotEncoderOffset = -0.120361;

            public static double getEncoderOffset() {
                return RioConstants.isSwerveBot() ? kSwerveBotEncoderOffset : kRacingBotEncoderOffset;
            }

            public static final SwerveModuleConstants ModuleConstants = ConstantCreator.createModuleConstants(
                kSteerMotorId, kDriveMotorId, kEncoderId, getEncoderOffset(), kXPos, kYPos, kInvertLeftSide);
        }

        // Back Right
        public static final class BackRight {
            public static final int kDriveMotorId = 16;
            public static final int kSteerMotorId = 17;
            public static final int kEncoderId = 23;

            public static final double kXPos = -kWheelBase / 2.0;
            public static final double kYPos = -kTrackWidth / 2.0;

            public static final double kSwerveBotEncoderOffset = 0.098877;
            public static final double kRacingBotEncoderOffset = -0.045166;

            public static double getEncoderOffset() {
                return RioConstants.isSwerveBot() ? kSwerveBotEncoderOffset : kRacingBotEncoderOffset;
            }

            public static final SwerveModuleConstants ModuleConstants = ConstantCreator.createModuleConstants(
                kSteerMotorId, kDriveMotorId, kEncoderId, getEncoderOffset(), kXPos, kYPos, kInvertRightSide);
        }

        public static final class Kinematics {
            public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
                    new Translation2d(FrontLeft.kXPos, FrontLeft.kYPos),
                    new Translation2d(FrontRight.kXPos, FrontRight.kYPos),
                    new Translation2d(BackLeft.kXPos, BackLeft.kYPos),
                    new Translation2d(BackRight.kXPos, BackRight.kYPos));
        }
    }
    
    public static class ShooterConstants {
        public static final int[] kShooterMotorIDs = {48, 49};
        public static final int[] kHolderMotorIDs = {33, 34};

        // how many rotations per second to output when shooting speaker
        public static final double kSpeakerShooterPower = 45;

        // how many rotations per second to output when shooting amp
        public static final double kAmpShooterPower = 6;
        
        // how many rotations per second to output when shooting to pass
        public static final double kSpeakerPassShooterPower = 65;

        public static final double kSpeakerPassShooterOffset = 0.65;

        // how many rotations per second to output when intaking
        public static final double kMaxIntakePower = -15;

        // how long to activate motors when automatically shooting
        public static final double kShooterTimeout = 0.5;

        // tolerance for when to start holder motors
        public static final double kVelocityTolerance = 0.2;

        // the minimum axis value to activate intake
        public static final double kMinAxisValue = 0.25;

        // closed-loop configs
        public static final Slot0Configs kShooterConfigs = new Slot0Configs().withKP(0.15)
                                                                   .withKS(0.09).withKV(0.11748);
        public static final Slot0Configs kHolderConfigs = new Slot0Configs().withKP(0.13).withKV(0.11047);
    }

    public static class AmpConstants {
        public static final int kMotorID = 8;
        public static final int kEncoderID = 9;
        // The PID constants
        public static final double kP = 70;
        public static final double kI = 0.0;
        public static final double kD = 0.02;
        public static final double kG = -0.26;

        // Trapezoid profile constants
        public static final double kMaxVel = 2.0; // rotations per second
        public static final double kMaxAcc = 4.0; // rotations per second^2

        public static final double kTolerance = 0.05;
        
        public static final double kUpPIDSetpoint = 0.135498;
        public static final double kUpHalfPidSetpoint = 0.045;
        public static final double kDownPIDSetpoint = -0.222168;
        public static final double kOffset = -0.279541;
    }

    public static class LimeLightConstants {
        // The default age limit for staleness of target data we read from limelight
        public static final long kDefaultLimeLightStalenessLimit = 110 * 1000 /*microseconds per millisecond */;
    }

    public static class AprilTagConstants {
        // Multiplier for the Snap point distance for each AprilTag (set to 1.0 by default) for the Y axis
        public static final double kAprilTagSnapPointYMultiplier = 1.0;
        // Multiplier for the Snap point distance for each AprilTag(set to 1.0 by default) for the X axis
        public static final double kAprilTagSnapPointXMultiplier = 1.0;
        // Constants to use rather than repetitive code for stage snap points
        public static final Pose2d kStageSnapPointCenter = new Pose2d(0.47, 0.0, Rotation2d.fromDegrees(0));
    }
    
    public static class AutoFireConstants {
        public static final double kAutoFireDistance = 1.5;

        public static final double kAutoFireTolerance = 0.2;

        public static final double kAutoFireAngleTolerance = 0.5;
    }
}