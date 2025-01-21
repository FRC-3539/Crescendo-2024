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

public class AutonNoteTrackCommand extends Command {
	double speedMultiplier = DrivetrainConstants.speedMultiplier;
	double rotationSpeedMultiplier = DrivetrainConstants.rotationSpeedMultiplier;
	double maxRotationalVelocity = RobotContainer.drivetrainSubsystem.maxRotationalVelocity;
	double maxVelocity = RobotContainer.drivetrainSubsystem.maxVelocity;
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
	@Override
	public void execute() {
		SwerveRequest request = new SwerveRequest.Idle();

		var target = RobotContainer.visionSubsystem.getBestFrontNote();
		if (target != null & !RobotContainer.intakeSubsystem.getBackSensor()
				& !RobotContainer.intakeSubsystem.getFrontSensor() & !RobotContainer.intakeSubsystem.getChamberSensor()
				& !RobotContainer.shooterSubsystem.getShooterSensor()) {
			noNoteCounter = 0;
			RobotContainer.ledSubsystem.setNoteTracking(true);

			double noteTrackSpeedMultiplier = 0.5;
			var angleToTarget = -target.getYaw() * Math.PI / 180;
			rotationController.setSetpoint(
					RobotContainer.drivetrainSubsystem.getPose2d().getRotation().getRadians() + angleToTarget);
			request = driveRobotCentric.withVelocityX(maxVelocity * noteTrackSpeedMultiplier).withVelocityY(0);
			driveRobotCentric.withRotationalRate(rotationController
					.calculate(RobotContainer.drivetrainSubsystem.getPose2d().getRotation().getRadians(), 0.02)
					* maxRotationalVelocity * .3).withRotationalDeadband(0);
			RobotContainer.drivetrainSubsystem.applyRequest(request);

		} else {
			noNoteCounter++;
			RobotContainer.ledSubsystem.setNoteTracking(false);
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		RobotContainer.ledSubsystem.setNoteTracking(false);
		RobotContainer.drivetrainSubsystem.applyRequest(new SwerveRequest.Idle());
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return RobotContainer.intakeSubsystem.getBackSensor() || RobotContainer.intakeSubsystem.getFrontSensor()
				|| RobotContainer.intakeSubsystem.getChamberSensor()
				|| RobotContainer.shooterSubsystem.getShooterSensor() || noNoteCounter > 10;
	}
}
