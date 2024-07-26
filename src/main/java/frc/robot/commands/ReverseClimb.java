// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LedSubsystem;

public class ReverseClimb extends Command {
	/** Creates a new independantClimbLeft. */
	public ReverseClimb() {
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		LedSubsystem.setReverseClimbing(true);

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		ClimberSubsystem.setLeftClimbMotorVoltage(-12);
		ClimberSubsystem.setRightClimbMotorVoltage(-12);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		ClimberSubsystem.setLeftClimbMotorVoltage(0);
		ClimberSubsystem.setRightClimbMotorVoltage(0);
		LedSubsystem.setReverseClimbing(false);

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
