package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class AutonomousConstants {
        public static final PIDConstants REEF_ASISST = new PIDConstants(0, 0);
        public static final PIDConstants ALGE_AIM_ASISST = new PIDConstants(0, 0);

    }

    public static final class ArmAngleChangeConstants {
        public static final int ArmAngleChange_Motor_ID =41;
        public static final double INTAKE_POSITION = -15;
        public static final double L2_L3_ANGLE_POSITION = 5;
        public static final double  KP_POSITION_PID = 0.2;
        public static final double  KI_POSITION_PID = 0;
        public static final double  KD_POSITION_PID = 0;
        
    }


    public static final class ElevatorConstants {
        public static final double KP_POSITION_PID = 0.2;
        public static final double L2_POSITION = 12;
        public static final double L3_POSITION = 42;
        public static final double L4_POSITION = 46;
        public static final int ELEVATOR_MOTOR1_ID = 0;
        public static final int ELEVATOR_MOTOR2_ID = 0;

        
    }
    
    public static final class IntakeConstants {
        public static final int FRONT_MOTOR_ID = 0;
        public static final int BACK_MOTOR_ID = 0;
    }

    public static final class AprilTagConstants {
        public static final Translation3d cam1Positoin = new Translation3d(0,0,0);
        public static final Translation3d cam2Positoin = new Translation3d(0,0,0);



    }
    public static final class Swerve {
        public static final Matrix<N3, N1> poseStdDevs = VecBuilder.fill(0.1, 0.1, 0.05);   // Vision trust factors
        public static final Matrix<N3, N1> odomStdDevs = VecBuilder.fill(0.2, 0.2, 0.1);    // Odometry trust factors


        
        public static final int pigeonID = 62;

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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-20.56640625);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
            }
            
            /** Front Right Module - Module 1 */
            public static final class Mod1 { //TODO: This must be tuned to specific robot
                public static final int driveMotorID = 2;
                public static final int angleMotorID = 12;
                public static final int canCoderID = 22;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-61.083984375);
                public static final SwerveModuleConstants constants = 
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
                }
                
                /** Back Left Module - Module 2 */
                public static final class Mod2 { //TODO: This must be tuned to specific robot
                    public static final int driveMotorID = 3;
                    public static final int angleMotorID = 13;
                    public static final int canCoderID = 23;
                    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-130.25390625);
                    public static final SwerveModuleConstants constants = 
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
                }
                
                /** Back Right Module - Module 3 */
                public static final class Mod3 { //TODO: This must be tuned to specific robot
                    public static final int driveMotorID = 4;
                    public static final int angleMotorID = 14;
                    public static final int canCoderID = 24;
                    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-9.228515625);
                    public static final SwerveModuleConstants constants = 
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
                }
    }
    public static final class  LEDConstants {
        public static final int LED_PORT = 0;
        public static final int LED_LENGTH = 60;
        
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final PIDConstants translationConstants = new PIDConstants(10.0, 0.0, 0);
        public static final PIDConstants rotationConstants = new PIDConstants(10.0, 0.0, 0);
    }

}
