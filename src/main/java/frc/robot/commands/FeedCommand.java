// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.ShooterConstants;

public class FeedCommand extends Command {

	private double shootSpeed;

	public FeedCommand(double shootSpeed) {
		this.shootSpeed = shootSpeed;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		RobotContainer.shooterSubsystem.setFeedMotorRPS(-ShooterConstants.feedDps / 12.0);
		RobotContainer.intakeSubsystem.setchamberVoltage(.5);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		RobotContainer.shooterSubsystem.setFeedMotorVoltage(0);
		RobotContainer.intakeSubsystem.setchamberVoltage(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
