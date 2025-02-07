package frc.robot;

import java.nio.channels.ConnectionPendingException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autoTelopCommands.ReefAsisst;
import frc.robot.commands.condition.CondionalArmChangeCommand;
import frc.robot.commands.condition.OutL1Command;
import frc.robot.commands.condition.OutL2Command;
import frc.robot.commands.condition.OutL3Command;
import frc.robot.commands.condition.OutL4Command;
import frc.robot.subsystems.*;
import frc.robot.vision.AprilTagVision;
import frc.robot.vision.ObjectDetection;

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
    public static final XboxController xboxController = new XboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */


    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Elevator elevator = new Elevator();
    public static final IntakeAlgeaChangAngle intakeAlgeaChangAngle = new IntakeAlgeaChangAngle();
    public static final IntakeAlgea intake = new IntakeAlgea();
    public static final ArmAngleChange armAngleChange = new ArmAngleChange();
    public static final Arm arm = new Arm();
    public static final ObjectDetection objectDetection = new ObjectDetection();
    //public static final Climb climb = new Climb();
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
        commandXBoxController.start().whileTrue(s_Swerve.zeroHeadingCommand());
       commandXBoxController.a().whileTrue(L1());
       commandXBoxController.b().whileTrue(L2());
       commandXBoxController.y().whileTrue(L3());
       commandXBoxController.x().whileTrue(L4());
       commandXBoxController.rightBumper().whileTrue(coralIntake());


       commandXBoxController.rightTrigger().whileTrue(new ReefAsisst
       (
        () -> driver.getRawAxis(translationAxis), 
        () -> driver.getRawAxis(strafeAxis), 
        () -> driver.getRawAxis(rotationAxis)));


       commandXBoxController.leftBumper().whileTrue(intakeAlgeaChangAngle.algeaIntakeCommand().alongWith(intake.intakeCommand()));
       commandXBoxController.leftTrigger().whileTrue(intake.OutakeCommand().alongWith(intakeAlgeaChangAngle.algeaOutakeCommand()));


    }
    public Command ElevatorTest(){
        return elevator.elevatorCommand();
    }
    public Command coralIntake() {
        return armAngleChange.setIntakePositionCommand()
        .alongWith(arm.intakCommand());
    }
    public Command L1(){
       return armAngleChange.setL1PositionCommand()
       .alongWith(new OutL1Command());
    }

    public Command L2(){
        return elevator.elevatorL2Command()
        .alongWith(armAngleChange.setL2L3PositionCommand())
        .alongWith(new OutL2Command());
    }

    public Command L3(){
            return elevator.elevatorL3Command()
            .alongWith(armAngleChange.setL2L3PositionCommand())
            .alongWith(new OutL3Command());
          }
    
        public Command L4(){
            return elevator.elevatorL4Command()
            .alongWith(new CondionalArmChangeCommand()
            .andThen(armAngleChange.setL4PositionCommand()))
            .alongWith(new OutL4Command());
        }
        
        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        private void setDefaultCommands() {
    
            arm.setDefaultCommand(arm.StopShootCommand());
    
            armAngleChange.setDefaultCommand(armAngleChange.zeroPositionCommad());
    
            elevator.setDefaultCommand(elevator.zeroPositionCommand());
    
            intake.setDefaultCommand(intake.stopMotorsCommand());
            
            intakeAlgeaChangAngle.setDefaultCommand(intakeAlgeaChangAngle.zeroPositionAlgeaCommand());
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
            NamedCommands.registerCommand("L3Auto", L3());
            
            
    }


    public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

}
