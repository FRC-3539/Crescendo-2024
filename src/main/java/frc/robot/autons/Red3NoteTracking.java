// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AngleShooterCommand;
import frc.robot.commands.AutonNoteTrackCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.HomePositionCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReturnToPathCommand;
import frc.robot.commands.RevUpCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.constants.ShooterConstants;

import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory;
import org.frcteam3539.Byte_Swerve_Lib.io.BBMPLoader;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Red3NoteTracking extends SequentialCommandGroup {
	BBMPLoader loader = new BBMPLoader("/home/lvuser/profiles/Red3NoteTracking.txt", true);
	Trajectory[] trajectories = loader.getTrajectories();
	private Command[] sequence = {
		new InstantCommand(() -> RobotContainer.drivetrainSubsystem.seedFieldRelative(loader.getFirstTrajectory())),
		new ParallelCommandGroup(
			new RevUpCommand(false, ShooterConstants.shootDps).withTimeout(15),
			new SequentialCommandGroup(
				// Go pick up second note
				new ParallelCommandGroup(
					new WaitCommand(0.4)
						.andThen(new ShootCommand().withTimeout(1)),
					new WaitCommand(0.2)
						.andThen(new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, trajectories[0]))
						.andThen(new AutonNoteTrackCommand().withTimeout(1))
						.andThen(new ReturnToPathCommand(RobotContainer.drivetrainSubsystem, trajectories[1])),
					new WaitCommand(2.5)
						.andThen(new IntakeCommand(true, IntakeMode.FRONT).withTimeout(3))
				),
				// Return and shoot second note
				new ParallelCommandGroup(
					new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, trajectories[1]).andThen(new WaitCommand(0.3)).andThen(new ShootCommand().withTimeout(.5))
						.andThen(new HomePositionCommand()),
					new WaitCommand(.5)
						.andThen(new AngleShooterCommand(-25))
						
				),
				// Go pick up third note
				new ParallelCommandGroup(
					new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, trajectories[2])
						.andThen(new AutonNoteTrackCommand().withTimeout(1))
						.andThen(new ReturnToPathCommand(RobotContainer.drivetrainSubsystem, trajectories[3])),
					new WaitCommand(2)
						.andThen(new IntakeCommand(true, IntakeMode.FRONT).withTimeout(5))
				),
				// Return and shoot third note
				new ParallelCommandGroup(
					new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, trajectories[3]).andThen(new WaitCommand(0.1)).andThen(new ShootCommand().withTimeout(1)),
					new WaitCommand(0.5).andThen(new AngleShooterCommand(-24.5))
				)
			)
		)
	};

	/** Creates a new RedShootDrive. */
	public Red3NoteTracking() {
		addCommands(sequence);
	}
}