package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.vision.AprilTagVision;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;


import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public class Swerve extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator; // Kalman-based pose estimator
    private final SwerveModule[] mSwerveMods;
    private final Pigeon2 gyro;
    private final Field2d m_field;
    

    private final AprilTagVision visionEstimator;

    public Swerve() {
        try{
            RobotConfig config = RobotConfig.fromGUISettings();
            
            // Configure AutoBuilder
            AutoBuilder.configure(
                this::getPose, 
                this::resetPose, 
                this::getSpeeds, 
                this::driveRobotRelative, 
                new PPHolonomicDriveController(
              Constants.AutoConstants.translationConstants,
              Constants.AutoConstants.rotationConstants
            ),
            config,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
          );
        }catch(Exception e){
          DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
        


        
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        
        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);
        
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)

        };

        
        
        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(), // Initial pose, usually (0,0) for autonomous start
            Constants.Swerve.poseStdDevs,   // Standard deviations for vision
            Constants.Swerve.odomStdDevs    // Standard deviations for odometry
        );

        visionEstimator = new AprilTagVision(); // Initialize vision estimator




    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                , Constants.Swerve.robotToSwerve
        );
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){

            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public ChassisSpeeds getSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getGyroYaw();
    }

    public void setHeading(Rotation2d heading){
        gyro.setYaw(0);    
    }

    public void zeroHeading(){
        gyro.setYaw(0);
    }

    public Command zeroHeadingCommand(){
        return this.run(() -> zeroHeading());
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        targetSpeeds.omegaRadiansPerSecond = -targetSpeeds.omegaRadiansPerSecond;
        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds, Constants.Swerve.robotToSwerve);
        setStates(targetStates);
      }

    public void setStates(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Swerve.maxSpeed);
    
        for (int i = 0; i < mSwerveMods.length; i++) {
          mSwerveMods[i].setDesiredState(targetStates[i], false);
        }
      }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){

        m_field.setRobotPose(getPose());
        // Update the Kalman filter with odometry data
        poseEstimator.update(getGyroYaw(), getModulePositions());
        System.out.println("left: " + RobotContainer.aprilTag.leftGetId() + " right: " + RobotContainer.aprilTag.rightGetId());

        // Get vision pose estimate and update if available
        Optional<Pose2d> visionPoseEstimate = visionEstimator.getEstimatedGlobalPose(getPose());
        if (visionPoseEstimate.isPresent()) {
            // poseEstimator.addVisionMeasurement(
            //     visionPoseEstimate.get(),
            //     edu.wpi.first.wpilibj.Timer.getFPGATimestamp()
            // );
        }

        // Update SmartDashboard for debugging
        SmartDashboard.putString("Robot Pose", getPose().toString());
        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }


    }
}
