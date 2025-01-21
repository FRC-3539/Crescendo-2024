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
import org.frcteam3539.Byte_Swerve_Lib.io.BBMPLoader;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Blue6Piece extends SequentialCommandGroup {
	BBMPLoader loader = new BBMPLoader("/home/lvuser/profiles/Blue6Piece.txt", false);

	private Command[] sequence = {
		new InstantCommand(() -> RobotContainer.drivetrainSubsystem.seedFieldRelative(loader.getFirstTrajectory())),
		new ParallelCommandGroup(
			new RevUpCommand(false, ShooterConstants.shootDps).withTimeout(15),
			new SequentialCommandGroup(
				// Go pick up second note
				new ParallelCommandGroup(
					new WaitCommand(0.4)
						.andThen(new ShootCommand().withTimeout(1)),
		
						new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, loader.getNextTrajectory())
						.andThen(new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, loader.getNextTrajectory()))
						.andThen(new AutonNoteTrackCommand().withTimeout(1))
						.andThen(new ReturnToPathCommand(RobotContainer.drivetrainSubsystem, loader.getNextTrajectory())),
					new WaitCommand(2.67)
						.andThen(new IntakeCommand(true, IntakeMode.FRONT).withTimeout(4))
				),
				// Return and shoot second note
				new ParallelCommandGroup(
					new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, loader.getCurrentTrajectory()),
					new WaitCommand(1.45)
						.andThen(new AngleShooterCommand(-25.5)),
					new WaitCommand(1.70)
						.andThen(new ShootCommand().withTimeout(1))
						.andThen(new HomePositionCommand())
				),
				// Go pick up third note
				new ParallelCommandGroup(
					new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, loader.getNextTrajectory())
						.andThen(new AutonNoteTrackCommand().withTimeout(1))
						.andThen(new ReturnToPathCommand(RobotContainer.drivetrainSubsystem, loader.getNextTrajectory())),
					new WaitCommand(1.57)
						.andThen(new IntakeCommand(true, IntakeMode.FRONT).withTimeout(5))
				),
				// Return and shoot third note
				new ParallelCommandGroup(
					new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, loader.getCurrentTrajectory()),
					new WaitCommand(1.28)
					.andThen(new AngleShooterCommand(-23)),
					new WaitCommand(1.57)
					.andThen(new ShootCommand().withTimeout(1))
					.andThen(new HomePositionCommand())

				),
				// Go pick up fourth note
				new ParallelCommandGroup(
					new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, loader.getNextTrajectory())
						.andThen(new AutonNoteTrackCommand().withTimeout(1)),
					(new IntakeCommand(true, IntakeMode.FRONT).withTimeout(5))
				),
				//shoot fourth note
				new ParallelCommandGroup(
					new WaitCommand(1.80)
					.andThen(new ShootCommand().withTimeout(1))
				),
				//pick up fifth note 
				new ParallelCommandGroup(
					new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, loader.getNextTrajectory())
						.andThen(new AutonNoteTrackCommand().withTimeout(1)),
					(new IntakeCommand(true, IntakeMode.FRONT).withTimeout(4))
				),
				//shoot fifth note and pick up sixth
				new ParallelCommandGroup(
					new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, loader.getNextTrajectory()),
					new WaitCommand(0.75)
					.andThen(new ShootCommand().withTimeout(1)),
					new WaitCommand(1.27).andThen(new IntakeCommand(true, IntakeMode.FRONT).withTimeout(5))
				),
				new ParallelCommandGroup(
					new WaitCommand(1.33)
					.andThen(new ShootCommand().withTimeout(1))
				)

			)
		)
	};

	/** Creates a new RedShootDrive. */
	public Blue6Piece() {
		addCommands(sequence);
	}
}