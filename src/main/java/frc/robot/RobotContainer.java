// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autons.*;
import frc.robot.commands.*;
import frc.robot.constants.*;
import frc.robot.generated.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.LogController;
import frc.robot.subsystems.LedSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	public static LogController logController = new LogController();

	public static DrivetrainConstants drivetrainConstants = new DrivetrainConstants();
	public static ClimberConstants climberConstants = new ClimberConstants();
	public static IDConstants idConstants = new IDConstants();
	public static IntakeConstants intakeConstants = new IntakeConstants();
	public static ShooterConstants shooterConstants = new ShooterConstants();
	public static VisionConstants visionConstants = new VisionConstants();

	public static TunerConstants tunerConstants = new TunerConstants();

	public static CommandSwerveDrivetrain drivetrainSubsystem = TunerConstantsComp.DriveTrain; // TunerConstants.DriveTrain
	public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

	public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
	public static LedSubsystem ledSubsystem = new LedSubsystem(true);
	public static VisionSubsystem visionSubsystem = new VisionSubsystem();
	public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public static CommandXboxController driverController = new CommandXboxController(1);

	public static CommandXboxController operatorController = new CommandXboxController(0);

	public static Trigger rightDriverTrigger = driverController.rightTrigger(0.5);
	public static Trigger rightDriverBumper = driverController.rightBumper();
	public static Trigger driverButtonA = driverController.a();
	public static Trigger driverButtonB = driverController.b();
	public static Trigger driverButtonX = driverController.x();

	public static Trigger rightOperatorBumper = operatorController.rightBumper();

	public static SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		putCommands();
		visionSubsystem.start();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate, or via the named factories in
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
	 * for {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
	 * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
	 * Flight joysticks}.
	 *
	 */
	public void putAutons(Alliance alliance) {

		if (alliance == Alliance.Blue) {
		} else {
		}
		SmartDashboard.putData(chooser);
	}
	private void configureBindings() {
		driverController.start().whileTrue(new ZeroGyroCommand());
		operatorController.rightBumper().whileTrue(new FeedCommand(5));
		drivetrainSubsystem.setDefaultCommand(new DriveCommand());
		operatorController.leftBumper().whileTrue(new ShootCommand(5, false));
		operatorController.leftTrigger().whileTrue(new IntakeCommand());
		operatorController.povDown().whileTrue(new ReverseClimbCommand());
		operatorController.povUp().whileTrue(new ClimbCommand());
	}

	public void putCommands() {

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return chooser.getSelected();
	}
}
