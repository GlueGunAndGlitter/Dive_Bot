package frc.robot;

import java.nio.channels.ConnectionPendingException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.IntakeCurrection;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.autoTelopCommands.ReefAsisst;
import frc.robot.commands.autoTelopCommands.ReefAsisstAuto;
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
    public static final IntakeAlgeaChangAngle intakeAlgeaChangAngle = new IntakeAlgeaChangAngle();
    public static final IntakeAlgea intake = new IntakeAlgea();
    public static final ArmAngleChange armAngleChange = new ArmAngleChange();
    public static final Arm arm = new Arm();
    public static final LEDSubsystem LED_SUBSYSTEM = new LEDSubsystem();
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
       //commandXBoxController.back().whileTrue(LowAlgea());
       commandXBoxController.leftTrigger().whileTrue(new ReefAsisst
       (
        s_Swerve,
        () -> driver.getRawAxis(translationAxis), 
        () -> driver.getRawAxis(strafeAxis), 
        () -> driver.getRawAxis(rotationAxis),
        isLeft).andThen(chsableLevel()));
       commandXBoxController.rightTrigger().whileTrue(new ReefAsisst
       (
        s_Swerve,
        () -> driver.getRawAxis(translationAxis), 
        () -> driver.getRawAxis(strafeAxis), 
        () -> driver.getRawAxis(rotationAxis),
        false));


       commandXBoxController.leftBumper().whileTrue(intakeAlgeaChangAngle.algeaIntakeCommand().alongWith(intake.intakeCommand()));
       commandXBoxController.back().whileTrue(intake.OutakeCommand().alongWith(intakeAlgeaChangAngle.algeaOutakeCommand()));




    }

    public Command coralIntake() {
        return armAngleChange.setIntakePositionCommand()
        .alongWith(new CoralIntake(arm)
        .andThen(new IntakeCurrection(arm)));
    }
   
    public Command LowAlgea(){
        return armAngleChange.setLowAlgeaCommand()
        .alongWith(elevator.elevatorL3Command());
        //.alongWith(arm.intakeOutPutCommand());
     }
     public Command highAlgea(){
        return elevator.elevatorL3Command()
        .alongWith(armAngleChange.setLowAlgeaCommand()
        .alongWith(arm.intakeOutPutCommand()));

        //   return elevator.elevatorL3Command()
        //   .andThen(armAngleChange.setLowAlgeaCommand()
        // .alongWith(arm.intakeOutPutCommand()));
     }

    public Command chsableLevel(){
        switch (level) {
            case 1:
               return L1(); 
            case 2:
                return L2(); 
            case 3:
                return L3(); 
            case 4:
                return L4(); 
            default:
                return L3();
        }
    }
    public Command L1(){
       return armAngleChange.setL1PositionCommand()
       .alongWith(new OutL1Command(arm));
    }

    public Command L2(){
        return elevator.elevatorL2Command()
        .alongWith(armAngleChange.setL2PositionCommand())
        .alongWith(new OutL2Command(arm));
    }

    public Command L3(){
            return elevator.elevatorL3Command()
            .alongWith(armAngleChange.setL3PositionCommand())
            .alongWith(new OutL3Command(arm));
          }
          public Command L3Auto(){
            return elevator.elevatorL3Command()
            .alongWith(armAngleChange.setL3PositionCommand())
            .alongWith(new WaitCommand(0.3).andThen(arm.normalOutCommand()));
          }
    public Command ZeroPosition(){
        return elevator.elevatorZeroCommand();
    }
    public Command StopElevatorArm(){
       return arm.stopCommand().alongWith(elevator.zeroPositionCommand());
    }
    
        public Command L4(){
            return elevator.elevatorL4Command()
            .alongWith(new CondionalArmChangeCommand()
            .andThen(armAngleChange.setL4PositionCommand()))
            .alongWith(new OutL4Command(arm));
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
            NamedCommands.registerCommand("StopElevatorArm", StopElevatorArm().withTimeout(1));
            NamedCommands.registerCommand("zeroelevatorAuto", ZeroPosition()); 
            NamedCommands.registerCommand("L3Auto", L3().withTimeout(1));
            NamedCommands.registerCommand("L3AutoUnlimeted", L3());

            NamedCommands.registerCommand("L4Auto", L4());
            NamedCommands.registerCommand("intakeAuto", coralIntake().withTimeout(2));
            NamedCommands.registerCommand("reefAssistAuto", new ReefAsisst
            (
                s_Swerve,
        () -> driver.getRawAxis(translationAxis), 
        () -> driver.getRawAxis(strafeAxis), 
        () -> driver.getRawAxis(rotationAxis),
        false));  
            
        NamedCommands.registerCommand("reefAssist60degAuto", new ReefAsisstAuto
            (
                s_Swerve,
        () -> driver.getRawAxis(translationAxis), 
        () -> driver.getRawAxis(strafeAxis), 
        () -> driver.getRawAxis(rotationAxis),
        false));  
            
            
    }


    public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

}
