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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;

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

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);


    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Elevator elevator = new Elevator();
    public static final Intake intake = new Intake();
    public static final ArmAngleChange armAngleChange = new ArmAngleChange();
    public static final Arm arm = new Arm();



    private final SendableChooser<Command> autoChooser;



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        setDefaultCommands();
        // Configure the button bindings
        configureButtonBindings();
;
        autoChooser = AutoBuilder.buildAutoChooser();
        

        SmartDashboard.putData("Auto Mode", autoChooser);
        registerCommand();
        
    }
    
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        //commandXBoxController.a().whileTrue(elevator.testUp());
        //commandXBoxController.a().whileTrue(elevator.testUp());
        //commandXBoxController.b().whileTrue(elevator.testDown());
        commandXBoxController.rightBumper().whileTrue(intake());
        // commandXBoxController.leftBumper().whileTrue(arm.outCommand());
        //commandXBoxController.y().whileTrue(armAngleChange.setIntakePositionCommand());
        //commandXBoxController.x().whileTrue(armAngleChange.l4AngleCommand());
        
        commandXBoxController.a().whileTrue(L1());
        commandXBoxController.x().whileTrue(L2());
        commandXBoxController.y().whileTrue(L3());
        commandXBoxController.b().whileTrue(L4());
    }
    public Command intake() {
        return armAngleChange.setIntakePositionCommand()
        .alongWith(arm.intakCommand());
    }
    public Command L1(){
       return armAngleChange.setL1PositionCommand()
       .alongWith(new WaitCommand(1)
       .andThen(arm.L1Output()));
    }

    public Command L2(){
        return elevator.elevatorL2Command()
        .alongWith(armAngleChange.setL2L3PositionCommand())
        .alongWith(new WaitCommand(1)
        .andThen(arm.intakCommand()));
    }

    public Command L3(){
            return elevator.elevatorL3Command()
            .alongWith(armAngleChange.setL2L3PositionCommand())
            .alongWith(new WaitCommand(3)
            .andThen(arm.intakCommand()));
          }
    
        public Command L4(){
            return elevator.elevatorL4Command()
            .alongWith(new WaitCommand(1.5)
            .andThen(armAngleChange.setL4PositionCommand()))
            .alongWith(new WaitCommand(4)
            .andThen(arm.outCommand()));
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
    
            s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                    s_Swerve, 
                    () -> -driver.getRawAxis(translationAxis), 
                    () -> -driver.getRawAxis(strafeAxis), 
                    () -> -driver.getRawAxis(rotationAxis), 
                    () -> robotCentric.getAsBoolean()
                )
            );
        
    
    
        }
    
    void registerCommand(){
            NamedCommands.registerCommand("nukePlzWork", L3());
    }


    public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

}
