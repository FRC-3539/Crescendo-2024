// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ClimbCommand extends Command {

	private double shootSpeed;

	public ClimbCommand() {
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		RobotContainer.climberSubsystem.setrightclimbMotorVoltage(2);
		RobotContainer.climberSubsystem.setleftclimbMotorVoltage(-2);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		RobotContainer.climberSubsystem.setrightclimbMotorVoltage(0);
		RobotContainer.climberSubsystem.setleftclimbMotorVoltage(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
