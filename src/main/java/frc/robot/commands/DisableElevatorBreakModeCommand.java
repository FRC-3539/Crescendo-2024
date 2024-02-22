// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DisableElevatorBreakModeCommand extends Command {
/** Creates a new DisableArmBreakModeCommand. */
public DisableElevatorBreakModeCommand() {
	// Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
@Override
public void initialize() {
	RobotContainer.shooterSubsystem.setElevatorBreakMode(false);
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
	RobotContainer.shooterSubsystem.setElevatorBreakMode(true);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
	return DriverStation.isEnabled();
}
}
