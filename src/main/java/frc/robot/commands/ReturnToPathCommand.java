package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import org.frcteam3539.Byte_Swerve_Lib.control.SimplePathBuilder;
import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory;
import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory.State;

public class ReturnToPathCommand extends Command {

	private final CommandSwerveDrivetrain drivetrain;
	private final Trajectory trajectory;
	public ReturnToPathCommand(CommandSwerveDrivetrain drivetrain, Trajectory trajectory) {
		this.drivetrain = drivetrain;
		this.trajectory = trajectory;

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		State s = trajectory.calculate(0);

		Trajectory t = new Trajectory(
				new SimplePathBuilder(drivetrain.getPose2d()).lineTo(s.getPathState().getPose2d()).build(),
				trajectory.getConstraints(), 0.02, 0, trajectory.getTrajectoryStartingVelocity());

		drivetrain.getFollower().follow(t);
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
