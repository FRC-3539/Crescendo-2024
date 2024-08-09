// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import org.frcteam3539.Byte_Swerve_Lib.control.PidConstants;
import org.frcteam3539.Byte_Swerve_Lib.control.PidController;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveCommand extends Command {
	Translation2d blueSpeakerCoordinate = new Translation2d(0, 5.55);
	Translation2d redSpeakerCoordinate = new Translation2d(0, 2.67);
	/** Creates a new DriveCommand. */
	private PidController rotationController;

	double maxVelocity = DriveSubsystem.maxVelocity;

	double maxRotationalVelocity = DriveSubsystem.maxRotationalVelocity;

	double rotationDeadband = maxRotationalVelocity * 0.02;
	private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
			.withDeadband(maxVelocity * 0.02).withRotationalDeadband(rotationDeadband) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

	private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
			.withDeadband(maxVelocity * 0.02).withRotationalDeadband(rotationDeadband) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

	public DriveCommand() {
		addRequirements(RobotContainer.driveSubsystem);

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
			LedSubsystem.setShootAligning(true);
			rotationController.setSetpoint(RobotContainer.driveSubsystem.getPose2d().getTranslation()
					.minus(DriveSubsystem.getOffsetTarget()).getAngle().getRadians());

			driveRobotCentric.withRotationalRate(
					rotationController.calculate(RobotContainer.driveSubsystem.getPose2d().getRotation().getRadians(),
							0.02) * maxRotationalVelocity * .3)
					.withRotationalDeadband(0);
			driveFieldCentric.withRotationalRate(
					rotationController.calculate(RobotContainer.driveSubsystem.getPose2d().getRotation().getRadians(),
							0.02) * maxRotationalVelocity * .3)
					.withRotationalDeadband(0);

		} else {
			LedSubsystem.setShootAligning(false);

		}
		if (RobotContainer.driverButtonB.getAsBoolean()) {
			var target = VisionSubsystem.getBestBackNote();
			if (target != null & !IntakeSubsystem.getBackSensor() & !IntakeSubsystem.getFrontSensor()
					& !IntakeSubsystem.getChamberSensor() & !ShooterSubsystem.getShooterSensor()) {
				LedSubsystem.setNoteTracking(true);
				double noteTrackSpeedMultiplier = 0.3;
				var angleToTarget = -target.getYaw() * Math.PI / 180;
				rotationController.setSetpoint(
						RobotContainer.driveSubsystem.getPose2d().getRotation().getRadians() + angleToTarget);
				request = driveRobotCentric.withVelocityX(-maxVelocity * noteTrackSpeedMultiplier).withVelocityY(0);
				driveRobotCentric.withRotationalRate(rotationController
						.calculate(RobotContainer.driveSubsystem.getPose2d().getRotation().getRadians(), 0.02)
						* maxRotationalVelocity * .3).withRotationalDeadband(0);
				driveFieldCentric.withRotationalRate(rotationController
						.calculate(RobotContainer.driveSubsystem.getPose2d().getRotation().getRadians(), 0.02)
						* maxRotationalVelocity * .3).withRotationalDeadband(0);

			} else {
				LedSubsystem.setNoteTracking(false);
			}
		} else if (RobotContainer.driverButtonX.getAsBoolean()) {
			var target = VisionSubsystem.getBestFrontNote();
			if (target != null & !IntakeSubsystem.getBackSensor() & !IntakeSubsystem.getFrontSensor()
					& !IntakeSubsystem.getChamberSensor() & !ShooterSubsystem.getShooterSensor()) {
				LedSubsystem.setNoteTracking(true);

				double noteTrackSpeedMultiplier = 0.3;
				var angleToTarget = -target.getYaw() * Math.PI / 180;
				rotationController.setSetpoint(
						RobotContainer.driveSubsystem.getPose2d().getRotation().getRadians() + angleToTarget);
				request = driveRobotCentric.withVelocityX(maxVelocity * noteTrackSpeedMultiplier).withVelocityY(0);
				driveRobotCentric.withRotationalRate(rotationController
						.calculate(RobotContainer.driveSubsystem.getPose2d().getRotation().getRadians(), 0.02)
						* maxRotationalVelocity * .3).withRotationalDeadband(0);
				driveFieldCentric.withRotationalRate(rotationController
						.calculate(RobotContainer.driveSubsystem.getPose2d().getRotation().getRadians(), 0.02)
						* maxRotationalVelocity * .3).withRotationalDeadband(0);

			} else {
				LedSubsystem.setNoteTracking(false);
			}
		} else {
			LedSubsystem.setNoteTracking(false);
		}
		DriveSubsystem.applyRequest(request);
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
}
