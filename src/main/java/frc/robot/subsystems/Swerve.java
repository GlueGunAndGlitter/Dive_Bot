package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.lib.LimelightHelpers;
import frc.lib.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.vision.AprilTagVision;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

public class Swerve extends SubsystemBase {
    private final SwerveDrivePoseEstimator poseEstimator; // Kalman-based pose estimator
    private final SwerveDrivePoseEstimator poseEstimatorNoVision; 

    private final SwerveModule[] mSwerveMods;
    private final Pigeon2 gyro;
    private final Field2d m_field;
    


    boolean doRejectUpdate;
   

    public Swerve() {
        try{
            RobotConfig config = RobotConfig.fromGUISettings();
            
            // Configure AutoBuilder
            AutoBuilder.configure(
                this::getPoseNoVision, 
                this::resetPoseNoVision, 
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

        poseEstimatorNoVision = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(), // Initial pose, usually (0,0) for autonomous start
            Constants.Swerve.poseStdDevs,   // Standard deviations for vision
            Constants.Swerve.odomStdDevs    // Standard deviations for odometry
        );



    


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
            positions[mod.moduleNumber].distanceMeters = -positions[mod.moduleNumber].distanceMeters;
        }
        return positions;
    }

    public ChassisSpeeds getSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPoseNoVision() {
        return poseEstimatorNoVision.getEstimatedPosition();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }
    public void resetPoseNoVision(Pose2d pose) {
        poseEstimatorNoVision.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeadingAngle(){
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return Rotation2d.fromDegrees(
                180 + 
                
                getPoseNoVision().getRotation().getDegrees());
        } 
        return getPoseNoVision().getRotation();
    }

    public Rotation2d getHeading(){
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return Rotation2d.fromDegrees(
                // 180 + 

                getPoseNoVision().getRotation().getDegrees());
        } 
        return getPoseNoVision().getRotation();
        
    }

    public void setHeading(Rotation2d heading){
        gyro.setYaw(heading.getDegrees());    
    }

    public void zeroHeading(){
        gyro.setYaw(180);
    }

    public Command zeroHeadingCommand(){
        return this.run(() -> gyro.setYaw(0));
    }

    public Command driveForword(){
        return this.run(() -> drive(new Translation2d(-1,0), 0, false , true));
    }
    public Command niggaStop(){
        return this.run(() -> drive(new Translation2d(0,0), 0, true , true));
    }

    public Command backWords(){
        return this.run(() -> drive(new Translation2d(1,0), 0, false , true));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        targetSpeeds.vxMetersPerSecond = -targetSpeeds.vxMetersPerSecond;
        targetSpeeds.vyMetersPerSecond = -targetSpeeds.vyMetersPerSecond;
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
        poseEstimator.update(getGyroYaw(), getModulePositions());

        doRejectUpdate = false;

        Optional<LimelightHelpers.PoseEstimate> mt2Left = RobotContainer.aprilTag.getEstimatedPose(Constants.LEFT_LIMELIGHT);
        Optional<LimelightHelpers.PoseEstimate> mt2Right = RobotContainer.aprilTag.getEstimatedPose(Constants.RIGHT_LIMELIGHT);

        // Optional<LimelightHelpers.PoseEstimate> mt2Right = RobotContainer.aprilTag.getEstimatedPose(Constants.RIGHT_LIMELIGHT); --------------------------------
        
        // Reject update if gyro angular velocity is too high
        if (Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 360) {
            doRejectUpdate = true;
        }
        
        // Check if either Limelight has detected a tag
        // SmartDashboard.putBoolean("isvalid", mt2Left.isPresent());
        boolean leftValid;
        if (mt2Left.isPresent()) {
            
             leftValid = mt2Left.get().tagCount > 0;
        }else{
            leftValid = false;
        }

        boolean rightValid;
        if (mt2Right.isPresent()) {
            
            rightValid = mt2Right.get().tagCount > 0;
       }else{
           rightValid = false;
       }

        // SmartDashboard.putBoolean("niggaRight", rightValid);

        if (!leftValid
         && !rightValid 
         ) {
            doRejectUpdate = true;
        }
        
        if (!doRejectUpdate) {
            Pose2d poseToUse;
            double timestampToUse;
        
            if (leftValid && rightValid) {
                // Use the average pose if both are valid
                Pose2d[] listPoses = {mt2Left.get().pose, mt2Right.get().pose};
                poseToUse = RobotContainer.aprilTag.averagePose(listPoses);
                timestampToUse = (mt2Left.get().timestampSeconds + mt2Right.get().timestampSeconds) / 2.0;
            } else if (leftValid) {
            
                poseToUse = mt2Left.get().pose;
                timestampToUse = mt2Left.get().timestampSeconds;
            } else {
                poseToUse = mt2Right.get().pose;
                timestampToUse = mt2Right.get().timestampSeconds;
            }
            poseEstimator.addVisionMeasurement(poseToUse, timestampToUse);
        }





        m_field.setRobotPose(getPose());
        // Update the Kalman filter with odometry data
        poseEstimatorNoVision.update(getGyroYaw(), getModulePositions());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }


    }
}
