// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedSubsystem;

public class AutoClimbCommand extends Command {
	/** Creates a new AutoClimbCommand. */
	public AutoClimbCommand() {
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		LedSubsystem.setClimbing(true);

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (DriveSubsystem.getRobotRoll().getDegrees() < -4) {
			ClimberSubsystem.setLeftClimbMotorVoltage(10);
			ClimberSubsystem.setRightClimbMotorVoltage(0);
		} else if (DriveSubsystem.getRobotRoll().getDegrees() > 4) {
			ClimberSubsystem.setLeftClimbMotorVoltage(0);
			ClimberSubsystem.setRightClimbMotorVoltage(10);
		} else {
			ClimberSubsystem.setLeftClimbMotorVoltage(10);
			ClimberSubsystem.setRightClimbMotorVoltage(10);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		LedSubsystem.setClimbing(false);

		ClimberSubsystem.setLeftClimbMotorVoltage(0);
		ClimberSubsystem.setRightClimbMotorVoltage(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
