// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import org.frcteam3539.Byte_Swerve_Lib.control.PidConstants;
import org.frcteam3539.Byte_Swerve_Lib.control.PidController;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.DrivetrainConstants;

public class DriveCommand extends Command {
	Translation2d blueSpeakerCoordinate = new Translation2d(0, 5.55);
	Translation2d redSpeakerCoordinate = new Translation2d(0, 2.67);
	/** Creates a new DriveCommand. */
	private PidController rotationController;

	double maxVelocity = RobotContainer.drivetrainSubsystem.maxVelocity;

	double maxRotationalVelocity = RobotContainer.drivetrainSubsystem.maxRotationalVelocity;

	double rotationDeadband = maxRotationalVelocity * 0.02;
	private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
			.withDeadband(maxVelocity * 0.02).withRotationalDeadband(rotationDeadband) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

	private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
			.withDeadband(maxVelocity * 0.02).withRotationalDeadband(rotationDeadband) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

	public DriveCommand() {
		addRequirements(RobotContainer.drivetrainSubsystem);

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

		double speedMultiplier = DrivetrainConstants.speedMultiplier;
		double rotationSpeedMultiplier = DrivetrainConstants.rotationSpeedMultiplier;

		if (RobotContainer.rightDriverTrigger.getAsBoolean()) // Turbo
		{
			speedMultiplier = DrivetrainConstants.turboSpeedMultiplier;
			rotationSpeedMultiplier = DrivetrainConstants.turboRotationSpeedMultiplier;
		}

		if (RobotContainer.rightDriverBumper.getAsBoolean()) { // Robot Centric
			request = driveRobotCentric
					.withVelocityX(-RobotContainer.driverController.getLeftY() * maxVelocity * speedMultiplier)
					.withVelocityY(-RobotContainer.driverController.getLeftX() * maxVelocity * speedMultiplier)
					.withRotationalRate(-RobotContainer.driverController.getRightX() * maxRotationalVelocity
							* rotationSpeedMultiplier)
					.withRotationalDeadband(rotationDeadband);
		} else {
			request = driveFieldCentric
					.withVelocityX(-RobotContainer.driverController.getLeftY() * maxVelocity * speedMultiplier)
					.withVelocityY(-RobotContainer.driverController.getLeftX() * maxVelocity * speedMultiplier)
					.withRotationalRate(-RobotContainer.driverController.getRightX() * maxRotationalVelocity
							* rotationSpeedMultiplier)
					.withRotationalDeadband(rotationDeadband);
		}
		if (RobotContainer.driverButtonA.getAsBoolean()) {
			RobotContainer.ledSubsystem.setShootAligning(true);
			rotationController.setSetpoint(RobotContainer.drivetrainSubsystem.getPose2d().getTranslation()
					.minus(RobotContainer.drivetrainSubsystem.getOffsetTarget()).getAngle().getRadians());

			driveRobotCentric.withRotationalRate(rotationController
					.calculate(RobotContainer.drivetrainSubsystem.getPose2d().getRotation().getRadians(), 0.02)
					* maxRotationalVelocity * .3).withRotationalDeadband(0);
			driveFieldCentric.withRotationalRate(rotationController
					.calculate(RobotContainer.drivetrainSubsystem.getPose2d().getRotation().getRadians(), 0.02)
					* maxRotationalVelocity * .3).withRotationalDeadband(0);

		} else {
			RobotContainer.ledSubsystem.setShootAligning(false);

		}
		if (RobotContainer.driverButtonB.getAsBoolean()) {
			var target = RobotContainer.visionSubsystem.getBestBackNote();
			if (target != null & !RobotContainer.intakeSubsystem.getBackSensor()
					& !RobotContainer.intakeSubsystem.getFrontSensor()
					& !RobotContainer.intakeSubsystem.getChamberSensor()
					& !RobotContainer.shooterSubsystem.getShooterSensor()) {
				RobotContainer.ledSubsystem.setNoteTracking(true);
				double noteTrackSpeedMultiplier = 0.3;
				var angleToTarget = -target.getYaw() * Math.PI / 180;
				rotationController.setSetpoint(
						RobotContainer.drivetrainSubsystem.getPose2d().getRotation().getRadians() + angleToTarget);
				request = driveRobotCentric.withVelocityX(-maxVelocity * noteTrackSpeedMultiplier).withVelocityY(0);
				driveRobotCentric.withRotationalRate(rotationController
						.calculate(RobotContainer.drivetrainSubsystem.getPose2d().getRotation().getRadians(), 0.02)
						* maxRotationalVelocity * .3).withRotationalDeadband(0);
				driveFieldCentric.withRotationalRate(rotationController
						.calculate(RobotContainer.drivetrainSubsystem.getPose2d().getRotation().getRadians(), 0.02)
						* maxRotationalVelocity * .3).withRotationalDeadband(0);

			} else {
				RobotContainer.ledSubsystem.setNoteTracking(false);
			}
		} else if (RobotContainer.driverButtonX.getAsBoolean()) {
			var target = RobotContainer.visionSubsystem.getBestFrontNote();
			if (target != null & !RobotContainer.intakeSubsystem.getBackSensor()
					& !RobotContainer.intakeSubsystem.getFrontSensor()
					& !RobotContainer.intakeSubsystem.getChamberSensor()
					& !RobotContainer.shooterSubsystem.getShooterSensor()) {
				RobotContainer.ledSubsystem.setNoteTracking(true);

				double noteTrackSpeedMultiplier = 0.3;
				var angleToTarget = -target.getYaw() * Math.PI / 180;
				rotationController.setSetpoint(
						RobotContainer.drivetrainSubsystem.getPose2d().getRotation().getRadians() + angleToTarget);
				request = driveRobotCentric.withVelocityX(maxVelocity * noteTrackSpeedMultiplier).withVelocityY(0);
				driveRobotCentric.withRotationalRate(rotationController
						.calculate(RobotContainer.drivetrainSubsystem.getPose2d().getRotation().getRadians(), 0.02)
						* maxRotationalVelocity * .3).withRotationalDeadband(0);
				driveFieldCentric.withRotationalRate(rotationController
						.calculate(RobotContainer.drivetrainSubsystem.getPose2d().getRotation().getRadians(), 0.02)
						* maxRotationalVelocity * .3).withRotationalDeadband(0);

			} else {
				RobotContainer.ledSubsystem.setNoteTracking(false);
			}
		} else {
			RobotContainer.ledSubsystem.setNoteTracking(false);
		}
		RobotContainer.drivetrainSubsystem.applyRequest(request);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

	private static Translation2d modifyJoystick(Translation2d joystick) {
		// Deadband
		Rotation2d rotation = joystick.getAngle();
		double distance = joystick.getNorm();

		double distanceModified = Math.copySign(Math.pow(distance, 3), distance);

		return new Translation2d(distanceModified, rotation);
	}
}
