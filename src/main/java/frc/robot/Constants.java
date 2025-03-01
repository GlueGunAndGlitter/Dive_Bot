// X = 0.36
// Y = 0.16


package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.156;
    


    public static final class AutonomousConstants {
        public static final PIDConstants REEF_ASISST = new PIDConstants(0, 0);
        public static final PIDConstants ALGE_AIM_ASISST = new PIDConstants(0, 0);

    }


    public static final class ArmConstants {
        public static final int MOTOR_ID = 61;
        public static final int BEAM_BREAKE_ID = 3;
        public static final double L1_SPEEED = 0.07;
        public static final double OUT_SPEEED = 0.5;
        public static final double INTAKE_SPEED = 0.15;
        
    }


    public static final class ArmAngleChangeConstants {
        public static final int ArmAngleChange_Motor_ID =32;
        public static final double INTAKE_POSITION = -2.8;
        public static final double L1_POSITION = 7.2;
        public static final double L2_ANGLE_POSITION = 3;
        public static final double L3_ANGLE_POSITION = 3;
        public static final double L4_POSITION = -37;
        public static final double LOW_ALGEA_POSITION = 7.52;
        public static final double  KP_POSITION_PID = 0.05;
        public static final double  KI_POSITION_PID = 0;
        public static final double  KD_POSITION_PID = 0;
        // 7
        public static final double CAN_OPEN_ARM = 9
        ;
        
    }


    public static final class ElevatorConstants {
        public static final double KP_POSITION_PID = 0.08;
        public static final double L2_POSITION = 5.7;
        public static final double L3_POSITION = 12.5;
        public static final double L4_POSITION = 13.6;
        public static final int ELEVATOR_MOTOR1_ID = 51;
        public static final int ELEVATOR_MOTOR2_ID = 52;

        
        public static final double DOWN_SPEED = 0.3;
        public static final double ZERO_POSITION = 0;
        
    }
    
    public static final class IntakeAlgeaChangAngle {
        public static final int INTAKE_ALGEA_ID = 42;
    }

    public static final class IntakeAlgeaConstants {

        public static final int INTAKE_ALGEA_ID_MOTOR = 41;

    }


    public static final class Swerve {
        public static final Matrix<N3, N1> poseStdDevs = VecBuilder.fill(0.1, 0.1, 0.05);   // Vision trust factors
        public static final Matrix<N3, N1> odomStdDevs = VecBuilder.fill(0.2, 0.2, 0.1);    // Odometry trust factors
        
                
        public static final int pigeonID = 7;

        public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific
        COTSTalonFXSwerveConstants.SDS.MK4i                              // robot
        .Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.725; //TODO: This must be tuned to specific robot
        public static final double wheelBase = 0.75; //TODO: This must be tuned to specific robot
        public static final Translation2d robotToSwerve = new Translation2d(0,0);
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double robotRadius = 436.46;



        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 5; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32/12; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51/12;
        public static final double driveKA = 0.27/12;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /** Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 21;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-40.4296875);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
                
            }
            
            /** Front Right Module - Module 1 */
            public static final class Mod1 { //TODO: This must be tuned to specific robot
                public static final int driveMotorID = 2;
                public static final int angleMotorID = 12;
                public static final int canCoderID = 22;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(70.224609375);
                public static final SwerveModuleConstants constants = 
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
                }
                
                /** Back Left Module - Module 2 */
                public static final class Mod2 { //TODO: This must be tuned to specific robot
                    public static final int driveMotorID = 3;
                    public static final int angleMotorID = 13;
                    public static final int canCoderID = 23;
                    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(13.095703125);
                    public static final SwerveModuleConstants constants = 
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
                }
                
                /** Back Right Module - Module 3 */
                public static final class Mod3 { //TODO: This must be tuned to specific robot
                    public static final int driveMotorID = 4;
                    public static final int angleMotorID = 14;
                    public static final int canCoderID = 24;
                    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-1.93359375);
                    public static final SwerveModuleConstants constants = 
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
                }
    }
    public static final class  LEDConstants {
        public static final int LED_PORT = 2; 
        public static final int LED_LENGTH = 20;
        
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final PIDConstants translationConstants = new PIDConstants(0.1, 0.0, 0);
        public static final PIDConstants rotationConstants = new PIDConstants(0, 0.0, 0);
    }


    public static final class ReefAsisstConstnt {

        static final Pose2d ID_6_TARGET_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(-60));
        static final Pose2d ID_6_TARGET_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(-60));

        static final Pose2d ID_7_TARGET_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        static final Pose2d ID_7_TARGET_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        static final Pose2d ID_8_TARGET_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(60));        
        static final Pose2d ID_8_TARGET_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(60));

        static final Pose2d ID_9_TARGET_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(120));
        static final Pose2d ID_9_TARGET_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(120));

        static final Pose2d ID_10_TARGET_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
        static final Pose2d ID_10_TARGET_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(180));

        static final Pose2d ID_11_TARGET_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(-120));
        static final Pose2d ID_11_TARGET_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(-120));
        
        static final Pose2d ID_17_TARGET_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(60));
        static final Pose2d ID_17_TARGET_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(60));

        static final Pose2d ID_18_TARGET_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        static final Pose2d ID_18_TARGET_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        static final Pose2d ID_19_TARGET_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(-60));
        static final Pose2d ID_19_TARGET_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(-60));

        static final Pose2d ID_20_TARGET_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(-120));
        static final Pose2d ID_20_TARGET_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(-120));

        static final Pose2d ID_21_TARGET_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
        static final Pose2d ID_21_TARGET_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
        
        static final Pose2d ID_22_TARGET_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(120));
        static final Pose2d ID_22_TARGET_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(120));


        public static final Map<Integer, Pose2d> LEFT_TARGETS = new HashMap<>();
        public static final Map<Integer, Pose2d> RIGHT_TARGETS = new HashMap<>();

        static {
            // Populate LEFT_TARGETS
            LEFT_TARGETS.put(6, ID_6_TARGET_LEFT);
            LEFT_TARGETS.put(7, ID_7_TARGET_LEFT);
            LEFT_TARGETS.put(8, ID_8_TARGET_LEFT);
            LEFT_TARGETS.put(9, ID_9_TARGET_LEFT);
            LEFT_TARGETS.put(10, ID_10_TARGET_LEFT);
            LEFT_TARGETS.put(11, ID_11_TARGET_LEFT);
            LEFT_TARGETS.put(17, ID_17_TARGET_LEFT);
            LEFT_TARGETS.put(18, ID_18_TARGET_LEFT);
            LEFT_TARGETS.put(19, ID_19_TARGET_LEFT);
            LEFT_TARGETS.put(20, ID_20_TARGET_LEFT);
            LEFT_TARGETS.put(21, ID_21_TARGET_LEFT);
            LEFT_TARGETS.put(22, ID_22_TARGET_LEFT);
        
            // Populate RIGHT_TARGETS
            RIGHT_TARGETS.put(6, ID_6_TARGET_RIGHT);
            RIGHT_TARGETS.put(7, ID_7_TARGET_RIGHT);
            RIGHT_TARGETS.put(8, ID_8_TARGET_RIGHT);
            RIGHT_TARGETS.put(9, ID_9_TARGET_RIGHT);
            RIGHT_TARGETS.put(10, ID_10_TARGET_RIGHT);
            RIGHT_TARGETS.put(11, ID_11_TARGET_RIGHT);
            RIGHT_TARGETS.put(17, ID_17_TARGET_RIGHT);
            RIGHT_TARGETS.put(18, ID_18_TARGET_RIGHT);
            RIGHT_TARGETS.put(19, ID_19_TARGET_RIGHT);
            RIGHT_TARGETS.put(20, ID_20_TARGET_RIGHT);
            RIGHT_TARGETS.put(21, ID_21_TARGET_RIGHT);
            RIGHT_TARGETS.put(22, ID_22_TARGET_RIGHT);
        }
    }
}

  


