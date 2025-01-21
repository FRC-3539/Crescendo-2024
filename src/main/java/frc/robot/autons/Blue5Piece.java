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
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RevUpCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.constants.ShooterConstants;
import org.frcteam3539.Byte_Swerve_Lib.io.BBMPLoader;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Blue5Piece extends SequentialCommandGroup {
	BBMPLoader loader = new BBMPLoader("/home/lvuser/profiles/Blue5Piece.txt", false);

	private Command[] sequence = {
			new InstantCommand(() -> RobotContainer.drivetrainSubsystem.seedFieldRelative(loader.getFirstTrajectory())),

			new ParallelCommandGroup(new RevUpCommand(false, ShooterConstants.shootDps).withTimeout(15),

					new SequentialCommandGroup(new WaitCommand(0.7), new ShootCommand().withTimeout(0.5)),
					new SequentialCommandGroup(new WaitCommand(3.6), new ShootCommand().withTimeout(0.5)),
					new SequentialCommandGroup(new WaitCommand(6.1), new ShootCommand().withTimeout(0.4)),
					new SequentialCommandGroup(new WaitCommand(9), new ShootCommand().withTimeout(0.5)),
					new SequentialCommandGroup(new WaitCommand(13.5), new AngleShooterCommand(-28.5).withTimeout(2.0)),
					new SequentialCommandGroup(new WaitCommand(14.75), new ShootCommand().withTimeout(1.0)),

					new SequentialCommandGroup(new WaitCommand(0.85),
							new IntakeCommand(true, IntakeMode.FRONT).withTimeout(2.0)),
					new SequentialCommandGroup(new WaitCommand(4.0),
							new IntakeCommand(true, IntakeMode.FRONT).withTimeout(2.0)),
					new SequentialCommandGroup(new WaitCommand(6.75),
							new IntakeCommand(true, IntakeMode.FRONT).withTimeout(3.0)),
					new SequentialCommandGroup(new WaitCommand(11.0),
							new IntakeCommand(true, IntakeMode.FRONT).withTimeout(5.0)),

					new SequentialCommandGroup(new WaitCommand(.5),
							new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem, loader.getNextTrajectory()),
							new FollowTrajectoryCommand(RobotContainer.drivetrainSubsystem,
									loader.getNextTrajectory())))};

	/** Creates a new RedShootDrive. */
	public Blue5Piece() {
		addCommands(sequence);
	}
}
