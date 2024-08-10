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
import frc.robot.commands.HomePositionCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RevUpCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.IntakeCommand.IntakeMode;
import frc.robot.constants.ShooterConstants;
import org.frcteam3539.Byte_Swerve_Lib.io.BBMPLoader;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Red3Piece extends SequentialCommandGroup {
	BBMPLoader loader = new BBMPLoader("/home/lvuser/profiles/Red3Piece.txt", false);

	private Command[] sequence = {
			new InstantCommand(() -> RobotContainer.driveSubsystem.seedFieldRelative(loader.getFirstTrajectory())),
			new ParallelCommandGroup(new RevUpCommand(false, ShooterConstants.shootDps).withTimeout(15),

					new SequentialCommandGroup(new WaitCommand(0.4), new ShootCommand().withTimeout(1)),

					new SequentialCommandGroup(new WaitCommand(6.75), new AngleShooterCommand(-25.5)),
					new SequentialCommandGroup(new WaitCommand(7.68), new ShootCommand().withTimeout(1)),
					new SequentialCommandGroup(new WaitCommand(8.25), new HomePositionCommand()),

					new SequentialCommandGroup(new WaitCommand(13), new AngleShooterCommand(-23)),
					new SequentialCommandGroup(new WaitCommand(14.75), new ShootCommand().withTimeout(1)),

					new SequentialCommandGroup(new WaitCommand(3.5),
							new IntakeCommand(true, IntakeMode.FRONT).withTimeout(4)),
					new SequentialCommandGroup(new WaitCommand(10.28),
							new IntakeCommand(true, IntakeMode.FRONT).withTimeout(4)),

					new SequentialCommandGroup(new WaitCommand(0.2),
							new FollowTrajectoryCommand(RobotContainer.driveSubsystem, loader.getNextTrajectory()),
							new FollowTrajectoryCommand(RobotContainer.driveSubsystem, loader.getNextTrajectory()),
							new FollowTrajectoryCommand(RobotContainer.driveSubsystem,
									loader.getNextTrajectory())))};
	/** Creates a new RedShootDrive. */
	public Red3Piece() {
		addCommands(sequence);
	}
}
