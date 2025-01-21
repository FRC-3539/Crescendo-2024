// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ReverseClimb extends Command {
	/** Creates a new independantClimbLeft. */
	public ReverseClimb() {
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		RobotContainer.ledSubsystem.setReverseClimbing(true);

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		RobotContainer.climberSubsystem.setLeftClimbMotorVoltage(-12);
		RobotContainer.climberSubsystem.setRightClimbMotorVoltage(-12);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		RobotContainer.climberSubsystem.setLeftClimbMotorVoltage(0);
		RobotContainer.climberSubsystem.setRightClimbMotorVoltage(0);
		RobotContainer.ledSubsystem.setReverseClimbing(false);

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
