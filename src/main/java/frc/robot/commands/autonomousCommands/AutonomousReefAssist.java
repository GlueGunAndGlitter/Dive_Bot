package frc.robot.commands.autonomousCommands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousReefAssist extends Command {    
    private final Swerve s_Swerve;    
    private Pose2d target;
    private int targetID;
    private final int widNumber;
    private final boolean isLeft;

    private final PIDController rotationPID = new PIDController(
        Constants.ReefAssistConstants.ROTATION_PID_VALUES.kP,
        Constants.ReefAssistConstants.ROTATION_PID_VALUES.kI,
        Constants.ReefAssistConstants.ROTATION_PID_VALUES.kD
    );

    private final PIDController yPID = new PIDController(
        Constants.ReefAssistConstants.Y_PID_CONSTANTS.kP,
        Constants.ReefAssistConstants.Y_PID_CONSTANTS.kI,
        Constants.ReefAssistConstants.Y_PID_CONSTANTS.kD
    );

    private final PIDController xPID = new PIDController(
        Constants.ReefAssistConstants.X_PID_CONSTANTS.kP,
        Constants.ReefAssistConstants.X_PID_CONSTANTS.kI,
        Constants.ReefAssistConstants.X_PID_CONSTANTS.kD
    );

    public AutonomousReefAssist(Swerve s_Swerve, int widNumber, boolean isLeft) {
        this.s_Swerve = s_Swerve;
        this.widNumber = widNumber;
        this.isLeft = isLeft;

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        Alliance alliance = DriverStation.getAlliance().get();
        
        if (alliance == Alliance.Red) {
            targetID = (widNumber == 6) ? 6 : widNumber + 6;
        } else {
            targetID = (widNumber == 6) ? 17 : widNumber + 17;
        }

        target = getTarget();
    }

    @Override
    public void execute() {
        Pose2d robotPose = s_Swerve.getPose();

        double error = target.getRotation().getDegrees() - Math.IEEEremainder(s_Swerve.getHeading().getDegrees(), 360);
        error = Math.IEEEremainder(error, 360); // Normalize error to -180 to 180
        
        double rotationOutput = rotationPID.calculate(error, 0);
        double yOutput = yPID.calculate(robotPose.getY(), target.getY());
        double xOutput = xPID.calculate(robotPose.getX(), target.getX());

        s_Swerve.drive(
            new Translation2d(xOutput, yOutput).times(Constants.Swerve.maxSpeed), 
            rotationOutput * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
        );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return atTolerance();
    }

    private boolean atTolerance() {
        Pose2d robotPose = s_Swerve.getPose();
        Transform2d error = robotPose.minus(target);

        return Math.abs(error.getX()) < Constants.ReefAssistConstants.X_TOLERANCE &&
               Math.abs(error.getY()) < Constants.ReefAssistConstants.Y_TOLERANCE &&
               Math.abs(error.getRotation().getDegrees()) < Constants.ReefAssistConstants.ROTATION_TOLERANCE;
    }

    private Pose2d getTarget() {
        return isLeft ? 
            Constants.ReefAssistConstants.LEFT_TARGETS.get(targetID) : 
            Constants.ReefAssistConstants.RIGHT_TARGETS.get(targetID);
    }
}
