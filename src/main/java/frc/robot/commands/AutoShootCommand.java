// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeConstants;

public class AutoShootCommand extends Command {

	/** Creates a new AutoShootCommand. */
	public AutoShootCommand() {
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		RobotContainer.ledSubsystem.setAutoShooting(true);
		// RobotContainer.shooterSubsystem
		// .setTopMotorSpeed(BBMath.getRps(ShooterConstants.shootDps,
		// ShooterConstants.shootWheelDiameter));
		// RobotContainer.shooterSubsystem
		// .setBottomMotorSpeed(BBMath.getRps(ShooterConstants.shootDps,
		// ShooterConstants.shootWheelDiameter));
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// if (RobotContainer.shooterSubsystem.getShooterSensor() == false) {
		// RobotContainer.shooterSubsystem.setFeedMotorSpeed(0);
		// }
		// if (!MathUtil.isNear(BBMath.getRps(ShooterConstants.shootDps,
		// ShooterConstants.shootWheelDiameter),
		// RobotContainer.shooterSubsystem.getTopMotorSpeed(), 5)) {
		// return;
		// }
		// if (!MathUtil.isNear(BBMath.getRps(ShooterConstants.shootDps,
		// ShooterConstants.shootWheelDiameter),
		// RobotContainer.shooterSubsystem.getBottomMotorSpeed(), 5)) {
		// return;
		// }
		// RobotContainer.shooterSubsystem
		// .setFeedMotorSpeed(BBMath.getRps(ShooterConstants.feedDps,
		// ShooterConstants.feedWheelDiameter));
		double angleToTarget = RobotContainer.shooterSubsystem.getEstimatedShooterAngle();
		RobotContainer.shooterSubsystem.setShooterAngle(angleToTarget);
		if (!RobotContainer.rightOperatorBumper.getAsBoolean()) {
			if (angleToTarget > -50) {
				RobotContainer.intakeSubsystem.setChamberMotorSpeed(IntakeConstants.intakeDps / 30);
			} else {
				RobotContainer.intakeSubsystem.setChamberMotorVoltage(0);
			}
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		RobotContainer.shooterSubsystem.setTopMotorVoltage(0);
		RobotContainer.shooterSubsystem.setBottomMotorVoltage(0);
		RobotContainer.shooterSubsystem.setFeedMotorVoltage(0);
		if (!RobotContainer.rightOperatorBumper.getAsBoolean()) {
			RobotContainer.intakeSubsystem.setChamberMotorVoltage(0);
		}
		RobotContainer.ledSubsystem.setAutoShooting(false);

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;

	}
}
