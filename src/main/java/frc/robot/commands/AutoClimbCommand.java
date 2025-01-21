// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoClimbCommand extends Command {
	/** Creates a new AutoClimbCommand. */
	public AutoClimbCommand() {
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		RobotContainer.ledSubsystem.setClimbing(true);

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (RobotContainer.drivetrainSubsystem.getRobotRoll().getDegrees() < -4) {
			RobotContainer.climberSubsystem.setLeftClimbMotorVoltage(10);
			RobotContainer.climberSubsystem.setRightClimbMotorVoltage(0);
		} else if (RobotContainer.drivetrainSubsystem.getRobotRoll().getDegrees() > 4) {
			RobotContainer.climberSubsystem.setLeftClimbMotorVoltage(0);
			RobotContainer.climberSubsystem.setRightClimbMotorVoltage(10);
		} else {
			RobotContainer.climberSubsystem.setLeftClimbMotorVoltage(10);
			RobotContainer.climberSubsystem.setRightClimbMotorVoltage(10);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		RobotContainer.ledSubsystem.setClimbing(false);

		RobotContainer.climberSubsystem.setLeftClimbMotorVoltage(0);
		RobotContainer.climberSubsystem.setRightClimbMotorVoltage(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
