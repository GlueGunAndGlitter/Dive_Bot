// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autoTelopCommands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Swerve;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class IntakeAsisst extends Command {


//   DoubleSupplier translationX;
//   DoubleSupplier translationY;
//   PIDController orizontalPID;
//   PIDController forwordBackwordsPID;
//   PIDController rotionPID;
//   DoubleSupplier rotationSup;
//   boolean targertIDChange = false;
//   double angle = 0;
//   Swerve swerve;

//   /** Creates a new ReefAsisst. */
//   public IntakeAsisst(Swerve swerve, DoubleSupplier translationX, DoubleSupplier translationY,DoubleSupplier rotationSup) {
//     orizontalPID = new PIDController(1, 0, 0);
//     forwordBackwordsPID = new PIDController(0.4, 0, 0);
//     rotionPID = new PIDController(0.02, 0, 0);

//     this.swerve = swerve;
//     this.translationX = translationX;
//     this.translationY = translationY;
//     this.rotationSup = rotationSup;
    
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(this.swerve);
//   }





//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double translationVal = MathUtil.applyDeadband(translationX.getAsDouble(), Constants.stickDeadband);
//     double strafeVal = MathUtil.applyDeadband(translationY.getAsDouble(), Constants.stickDeadband);
//     double rotationVal = -MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
//     double outputOrizontal;
//     double outputforwordBackwords;
//     double output;


//     if (!targertIDChange && RobotContainer.aprilTag.intakeCameraHasTarget()) {
//         angle = getAngle();
//         targertIDChange = true;
//     }
    
    
//     output = Math.IEEEremainder(angle - Math.IEEEremainder(swerve.getHeading().getDegrees(), 360),360) * 0.02;
    

//      outputOrizontal = orizontalPID.calculate(RobotContainer.aprilTag.intakeCameraGetY(),0.03);
//      outputforwordBackwords = forwordBackwordsPID.calculate(RobotContainer.aprilTag.intakeCameraGetX(), 0.53);
    

  
//     if (RobotContainer.aprilTag.intakeCameraHasTarget()) {
      
//       swerve.drive(
//         new Translation2d(outputforwordBackwords, outputOrizontal).times(Constants.Swerve.maxSpeed), 
//         -output * Constants.Swerve.maxAngularVelocity, 
//         false, 
//         true);
      
//     } else{
//       swerve.drive(
//         new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
//         -rotationVal * Constants.Swerve.maxAngularVelocity, 
//         true, 
//         true);
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   private double getAngle(){
//       if (RobotContainer.aprilTag.intakeCameraGetID() == 1 || RobotContainer.aprilTag.intakeCameraGetID() == 13) {
//         return 45;
//       }else if (RobotContainer.aprilTag.intakeCameraGetID() == 2 || RobotContainer.aprilTag.intakeCameraGetID() == 12) {
//         return -45;
//       }
//       return 45;
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//      return Math.abs(RobotContainer.aprilTag.intakeCameraGetY() - 0.16) < 0.03 && Math.abs(RobotContainer.aprilTag.intakeCameraGetX() - 0.34) < 0.15;
//   }
// }
