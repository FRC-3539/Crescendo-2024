package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory;

public class FollowTrajectoryCommand extends Command {

	private final CommandSwerveDrivetrain drivetrain;

	private final Trajectory trajectory;

	public FollowTrajectoryCommand(CommandSwerveDrivetrain drivetrain, Trajectory trajectory) {
		this.drivetrain = drivetrain;
		this.trajectory = trajectory;

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		drivetrain.getFollower().follow(trajectory);
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.getFollower().cancel();
	}

	@Override
	public boolean isFinished() {
		return drivetrain.getFollower().getCurrentTrajectory().isEmpty();
	}
}
