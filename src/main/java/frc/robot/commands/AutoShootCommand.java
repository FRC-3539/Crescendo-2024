// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootCommand extends Command {

	/** Creates a new AutoShootCommand. */
	public AutoShootCommand() {
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		LedSubsystem.setAutoShooting(true);
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
		double angleToTarget = ShooterSubsystem.getEstimatedShooterAngle();
		ShooterSubsystem.setShooterAngle(angleToTarget);
		if (!RobotContainer.rightOperatorBumper.getAsBoolean()) {
			
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		ShooterSubsystem.setTopMotorVoltage(0);
		ShooterSubsystem.setBottomMotorVoltage(0);
		ShooterSubsystem.setFeedMotorVoltage(0);
		if (!RobotContainer.rightOperatorBumper.getAsBoolean()) {
			IntakeSubsystem.setChamberMotorVoltage(0);
		}
		LedSubsystem.setAutoShooting(false);

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;

	}
}
