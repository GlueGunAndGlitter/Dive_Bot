package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RumbleForHalfSecend;
import frc.robot.commands.TeleopCommand.ActiveClimb;
import frc.robot.commands.TeleopCommand.ChoosableLevelCommand;
import frc.robot.commands.TeleopCommand.CoralIntake;
import frc.robot.commands.TeleopCommand.CoralOut;
import frc.robot.commands.TeleopCommand.GetOutAlgeHigh;
import frc.robot.commands.TeleopCommand.GetOutAlgeLow;
import frc.robot.commands.TeleopCommand.PreOpen;
import frc.robot.commands.TeleopCommand.TeleopSwerve;
import frc.robot.commands.TeleopCommand.LevelsCommands.L1Command;
import frc.robot.commands.TeleopCommand.LevelsCommands.L2Command;
import frc.robot.commands.TeleopCommand.LevelsCommands.L3Command;
import frc.robot.commands.TeleopCommand.LevelsCommands.L4Command;
import frc.robot.commands.autoTelopCommands.ReefAsisst;
import frc.robot.commands.autonomousCommands.AutonomousReefAssist;
import frc.robot.commands.autonomousCommands.Intake;
import frc.robot.commands.autonomousCommands.ReefAsisstWithPreOpen;
// import frc.robot.commands.autonomousCommands.AutonomousReefAssist;
import frc.robot.commands.autonomousCommands.ZeroRobotCommand;
import frc.robot.subsystems.*;
import frc.robot.vision.AprilTagVision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final Joystick driver = new Joystick(0);

    public static final CommandXboxController commandXBoxController = new CommandXboxController(0);
    public static final CommandXboxController soolyControler = new CommandXboxController(1);
    public static final XboxController xboxController = new XboxController(0);
    public static final XboxController soolyXboxControler = new XboxController(1);


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    public static int level = 1;
    public static boolean isLeft = false;

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Elevator elevator = new Elevator();
    // public static final IntakeAlgeaChangAngle intakeAlgeaChangAngle = new IntakeAlgeaChangAngle();
    // public static final IntakeAlgea intake = new IntakeAlgea();
    public static final ArmAngleChange armAngleChange = new ArmAngleChange();
    public static final Arm arm = new Arm();
    public static final LEDSubsystem LED_SUBSYSTEM = new LEDSubsystem();
    public static final Climb climb = new Climb();
    public static final AprilTagVision aprilTag = new AprilTagVision();



    private final SendableChooser<Command> autoChooser;



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        registerCommand();
        setDefaultCommands();
        
        // Configure the button bindings
        configureButtonBindings();
;
        autoChooser = AutoBuilder.buildAutoChooser();
        

        SmartDashboard.putData("Auto Mode", autoChooser);
        
        
    }
    
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        soolyControler.start().whileTrue(arm.stackCommand());

        commandXBoxController.start().whileTrue(s_Swerve.zeroHeadingCommand());
       commandXBoxController.rightBumper().toggleOnTrue(new CoralIntake(arm, armAngleChange).andThen(new RumbleForHalfSecend()));
    //    .alongWith(arm.setRumbleCoralInCommand().withTimeout(0.5)));
       commandXBoxController.rightTrigger().whileTrue((new ReefAsisst(s_Swerve).andThen(new CoralOut(arm).alongWith(s_Swerve.stopDrive())))
       .alongWith(new PreOpen(armAngleChange, elevator)));

       
    //    commandXBoxController.leftBumper().toggleOnTrue(new ArmRestOnMashpeh(armAngleChange, elevator));
    //    commandXBoxController.leftBumper().whileTrue(climb.rotateMotor());
        soolyControler.back().toggleOnTrue(armAngleChange.setClimPositionCommand().alongWith(new ActiveClimb()));
        soolyControler.rightTrigger().whileTrue(climb.rotateMotormunusCommand());                                

       commandXBoxController.a().whileTrue(new L1Command(armAngleChange, arm,elevator));
       commandXBoxController.back().whileTrue(new ChoosableLevelCommand(arm, armAngleChange, elevator));
       //commandXBoxController.b().whileTrue(new L3Command(armAngleChange, arm, elevator));
    //     commandXBoxController.b().whileTrue(new ReefAsisst
    //    (
    //     s_Swerve, armAngleChange).andThen(new L2Command(armAngleChange, arm, elevator)));

    //     commandXBoxController.y().whileTrue(new ReefAsisst
    //    (
    //     s_Swerve, armAngleChange).andThen(new L3Command(armAngleChange, arm, elevator)));

    //     commandXBoxController.x().whileTrue(new ReefAsisst
    //    (
    //     s_Swerve, armAngleChange).andThen(new L4Command(armAngleChange, arm, elevator)));

        commandXBoxController.b().whileTrue(new GetOutAlgeLow(elevator, arm, armAngleChange));
        commandXBoxController.y().whileTrue(new GetOutAlgeHigh(elevator, arm, armAngleChange));

    }



        
        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */


        private void setDefaultCommands() {
            
            climb.setDefaultCommand(climb.defaultCommand());

            arm.setDefaultCommand(arm.defaultCommand());
    
            armAngleChange.setDefaultCommand(armAngleChange.defaultCommand());
    
            elevator.setDefaultCommand(elevator.defaultCommand());
    
            // intake.setDefaultCommand(intake.defaultCommand());
            
            // intakeAlgeaChangAngle.setDefaultCommand(intakeAlgeaChangAngle.defaultCommand());
            s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                    s_Swerve, 
                    () -> driver.getRawAxis(translationAxis), 
                    () -> driver.getRawAxis(strafeAxis), 
                    () -> driver.getRawAxis(rotationAxis)
                )
            );
        
    
    
        }
    
    void registerCommand(){
            NamedCommands.registerCommand("L3AutPo", new L3Command(armAngleChange, arm, elevator));
            NamedCommands.registerCommand("L4Auto", new L4Command(armAngleChange, arm, elevator).withTimeout(1.5));
            NamedCommands.registerCommand("intakeAutonew12", new CoralIntake(arm, armAngleChange));
            NamedCommands.registerCommand("ZeroRobotAuto", new ZeroRobotCommand(armAngleChange, elevator));
            NamedCommands.registerCommand("ElevatorL4", elevator.elevatorL4Command());
            NamedCommands.registerCommand("IntakeNoPush", new Intake(arm, armAngleChange));
            

        
            NamedCommands.registerCommand("LeftReefAssistAuto3", new ReefAsisstWithPreOpen(s_Swerve, armAngleChange, 3, true, elevator)
            .andThen(arm.L4OutCommand().withTimeout(1.3).alongWith(s_Swerve.stopDrive().withTimeout(1.3))));
            NamedCommands.registerCommand("RightReefAssistAuto2", new ReefAsisstWithPreOpen(s_Swerve, armAngleChange, 2, false, elevator)
            .andThen(arm.L4OutCommand().withTimeout(1.3).alongWith(s_Swerve.stopDrive().withTimeout(1.3))));
            NamedCommands.registerCommand("LeftReefAssistAuto2", new ReefAsisstWithPreOpen(s_Swerve, armAngleChange, 2, true, elevator)
            .andThen(arm.L4OutCommand().withTimeout(1.3).alongWith(s_Swerve.stopDrive().withTimeout(1.3))));       
            NamedCommands.registerCommand("LeftReefAssistAuto4", new ReefAsisstWithPreOpen(s_Swerve, armAngleChange, 4, true, elevator)
            .andThen(arm.L4OutCommand().withTimeout(2.5).alongWith(s_Swerve.stopDrive().withTimeout(2.5))));
            NamedCommands.registerCommand("LeftReefAssistAuto6", new ReefAsisstWithPreOpen(s_Swerve, armAngleChange, 6, true, elevator)
            .andThen(arm.L4OutCommand().withTimeout(1.3).alongWith(s_Swerve.stopDrive().withTimeout(1.3))));
            NamedCommands.registerCommand("RightReefAssistAuto6", new ReefAsisstWithPreOpen(s_Swerve, armAngleChange, 6, false, elevator)
            .andThen(arm.L4OutCommand().withTimeout(1.3).alongWith(s_Swerve.stopDrive().withTimeout(1.3))));
            NamedCommands.registerCommand("LeftReefAssistAuto5", new ReefAsisstWithPreOpen(s_Swerve, armAngleChange, 5, true, elevator)
            .andThen(arm.L4OutCommand().withTimeout(1.3).alongWith(s_Swerve.stopDrive().withTimeout(1.3))));


            




        // () -> driver.getRawAxis(translationAxis), 

        // () -> driver.getRawAxis(strafeAxis), 
        // () -> driver.getRawAxis(rotationAxis),true));
            
        
            
    }


    public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

}
