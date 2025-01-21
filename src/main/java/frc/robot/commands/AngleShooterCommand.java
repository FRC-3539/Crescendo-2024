// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AngleShooterCommand extends Command {

	double angle;

	/** Creates a new AngleShooterCommand. */
	public AngleShooterCommand(double angle) {
		// Use addRequirements() here to declare subsystem dependencies..a
		this.angle = angle;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		RobotContainer.shooterSubsystem.setShooterAngle(angle);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		// System.out.println(angle + " " +
		// RobotContainer.shooterSubsystem.getShooterAngle());
		// RobotContainer.shooterSubsystem.setAngleMotorSpeed(
		// maxSpeed * RobotContainer.operatorController.getLeftY());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		// RobotContainer.shooterSubsystem.setAngleMotorSpeed(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return MathUtil.isNear(angle, RobotContainer.shooterSubsystem.getShooterAngle(), 1.5);
	}
}
