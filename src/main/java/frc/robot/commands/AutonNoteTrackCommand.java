// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.frcteam3539.Byte_Swerve_Lib.control.PidConstants;
import org.frcteam3539.Byte_Swerve_Lib.control.PidController;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutonNoteTrackCommand extends Command {
	double speedMultiplier = DrivetrainConstants.speedMultiplier;
	double rotationSpeedMultiplier = DrivetrainConstants.rotationSpeedMultiplier;
	double maxRotationalVelocity = DriveSubsystem.maxRotationalVelocity;
	double maxVelocity = DriveSubsystem.maxVelocity;
	int noNoteCounter = 0;

	private PidController rotationController;
	double rotationDeadband = maxRotationalVelocity * 0.05;

	private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
			.withDeadband(maxVelocity * 0.1).withRotationalDeadband(rotationDeadband) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	/** Creates a new AutonNoteTrackCommand. */
	public AutonNoteTrackCommand() {
		// Use addRequirements() here to declare subsystem dependencies.

		rotationController = new PidController(new PidConstants(DrivetrainConstants.AlignkP, 0, 0));

		rotationController.setInputRange(-Math.PI, Math.PI);
		rotationController.setOutputRange(-1, 1);
		rotationController.setContinuous(true);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
	@Override
	public void execute() {
		SwerveRequest request = idleRequest;

		var target = VisionSubsystem.getBestFrontNote();
		if (target != null & !IntakeSubsystem.getBackSensor() & !IntakeSubsystem.getFrontSensor()
				& !IntakeSubsystem.getChamberSensor() & !ShooterSubsystem.getShooterSensor()) {
			noNoteCounter = 0;
			LedSubsystem.setNoteTracking(true);

			double noteTrackSpeedMultiplier = 0.5;
			var angleToTarget = -target.getYaw() * Math.PI / 180;
			rotationController
					.setSetpoint(RobotContainer.driveSubsystem.getPose2d().getRotation().getRadians() + angleToTarget);
			request = driveRobotCentric.withVelocityX(maxVelocity * noteTrackSpeedMultiplier).withVelocityY(0);
			driveRobotCentric.withRotationalRate(
					rotationController.calculate(RobotContainer.driveSubsystem.getPose2d().getRotation().getRadians(),
							0.02) * maxRotationalVelocity * .3)
					.withRotationalDeadband(0);
			DriveSubsystem.applyRequest(request);

		} else {
			noNoteCounter++;
			LedSubsystem.setNoteTracking(false);
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		LedSubsystem.setNoteTracking(false);
		DriveSubsystem.applyRequest(idleRequest);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return IntakeSubsystem.getBackSensor() || IntakeSubsystem.getFrontSensor() || IntakeSubsystem.getChamberSensor()
				|| ShooterSubsystem.getShooterSensor() || noNoteCounter > 10;
	}
}
