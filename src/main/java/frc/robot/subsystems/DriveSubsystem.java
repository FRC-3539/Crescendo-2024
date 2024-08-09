// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.IDConstants;
import java.util.Arrays;
import org.frcteam3539.Byte_Swerve_Lib.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam3539.Byte_Swerve_Lib.control.PidConstants;
import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory;
import org.frcteam3539.Byte_Swerve_Lib.util.DrivetrainFeedforwardConstants;
import org.frcteam3539.Byte_Swerve_Lib.util.HolonomicFeedforward;

public class DriveSubsystem extends SwerveDrivetrain implements Subsystem {
	/** Creates a new DrivetrainSubsystem. */
	private static HolonomicMotionProfiledTrajectoryFollower follower;

	private static SwerveRequest swerveRequest = new SwerveRequest.Idle();

	static Translation2d blueSpeakerCoordinate = new Translation2d(0, 5.55);
	static Translation2d redSpeakerCoordinate = new Translation2d(0, 2.67);

	public static double velocityX = 0.0;
	public static double velocityY = 0.0;

	public static double maxVelocity = 0.0;
	public static double maxRotationalVelocity = 0.0;

	public static Pigeon2 pigeon = new Pigeon2(IDConstants.PigeonID, "canivore");

	public DriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);

		maxVelocity = modules[0].SpeedAt12VoltsMps;

		Translation2d[] moduleLocations = new Translation2d[ModuleCount];

		for (int i = 0; i < modules.length; i++) {
			moduleLocations[i] = new Translation2d(modules[i].LocationX, modules[i].LocationY);
			this.Modules[i].getDriveMotor().getConfigurator()
					.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(40).withSupplyCurrentLimitEnable(true)
							.withSupplyCurrentThreshold(40).withSupplyTimeThreshold(0)
							.withStatorCurrentLimit(modules[i].SlipCurrent).withStatorCurrentLimitEnable(true));
		}

		double dtRadius = new Translation2d().nearest(Arrays.asList(moduleLocations)).getDistance(new Translation2d());
		maxRotationalVelocity = (maxVelocity / dtRadius);

		DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
				DrivetrainConstants.TranslationkV, DrivetrainConstants.TranslationkA,
				DrivetrainConstants.TranslationkS);

		follower = new HolonomicMotionProfiledTrajectoryFollower(
				new PidConstants(DrivetrainConstants.TranslationkP, DrivetrainConstants.TranslationkI,
						DrivetrainConstants.TranslationkD),
				new PidConstants(DrivetrainConstants.RotationkP, DrivetrainConstants.RotationkI,
						DrivetrainConstants.RotationkD),
				new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));
	}

	public static HolonomicMotionProfiledTrajectoryFollower getFollower() {
		return follower;
	}

	public void seedFieldRelative(Trajectory trajectory) {
		seedFieldRelative(trajectory.calculate(0).getPathState().getPose2d());
	}

	public Pose2d getPose2d() {
		return m_odometry.getEstimatedPosition();
	}

	public static void applyRequest(SwerveRequest request) {
		swerveRequest = request;
	}

	public static void cancelPath() {
		follower.cancel();
	}

	public static void followPath(Trajectory t) {
		follower.follow(t);
	}

	public static Rotation2d getRobotRoll() {
		return Rotation2d.fromDegrees(
				BaseStatusSignal.getLatencyCompensatedValue(pigeon.getRoll(), pigeon.getAngularVelocityYDevice()));
	}
	public static Translation2d getOffsetTarget() {
		Translation2d offsetTarget;
		double noteSpeed = 16;
		double distanceToTarget = 0;
		if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
			distanceToTarget = Math.sqrt(Math.pow(
					RobotContainer.driveSubsystem.getPose2d().getTranslation().getDistance(redSpeakerCoordinate), 2)
					+ Math.pow(2.05, 2));
			double timeToTarget = distanceToTarget / noteSpeed;
			offsetTarget = new Translation2d(redSpeakerCoordinate.getX() - velocityX * timeToTarget,
					redSpeakerCoordinate.getY() - velocityY * timeToTarget);
		} else {
			distanceToTarget = Math.sqrt(Math.pow(
					RobotContainer.driveSubsystem.getPose2d().getTranslation().getDistance(blueSpeakerCoordinate), 2)
					+ Math.pow(2.05, 2));
			double timeToTarget = distanceToTarget / noteSpeed;
			offsetTarget = new Translation2d(blueSpeakerCoordinate.getX() - velocityX * timeToTarget,
					blueSpeakerCoordinate.getY() - velocityY * timeToTarget);
		}
		SmartDashboard.putNumberArray("/DriveTrain/OffsetTarget",
				new double[]{offsetTarget.getX(), offsetTarget.getY()});
		return offsetTarget;
	}

	public void log() {
		SmartDashboard.putNumber("/DriveTrain/RobotRoll", getRobotRoll().getDegrees());
		VisionSubsystem.publishPose2d("/DriveTrain/Pose", getPose2d());

		// Pose2d trajectory = follower.getLastState() != null
		// ? follower.getLastState().getPathState().getPose2d()
		// : new Pose2d(-1, -1, new Rotation2d(0));
		// Logger.recordOutput("/DriveTrain/Trajectory", trajectory);
	}

	SwerveRequest.RobotCentric autonRequest = new SwerveRequest.RobotCentric();
	SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
	@Override
	public void periodic() {

		velocityX = ChassisSpeeds.fromRobotRelativeSpeeds(this.getState().speeds,
				this.getPose2d().getRotation()).vxMetersPerSecond;
		velocityY = ChassisSpeeds.fromRobotRelativeSpeeds(this.getState().speeds,
				this.getPose2d().getRotation()).vyMetersPerSecond;

		SwerveRequest request = idleRequest;

		var driveSignalOpt = follower.update(getPose2d(), Timer.getFPGATimestamp(), Robot.kDefaultPeriod);

		if (follower.getLastState() != null) {
			VisionSubsystem.publishPose2d("/DriveTrain/PoseRequested",
					follower.getLastState().getPathState().getPose2d());
		} else {
			VisionSubsystem.publishPose2d("/DriveTrain/PoseRequested", new Pose2d());
		}

		// If we should be running a profile use those chassisspeeds instead.
		if (driveSignalOpt.isPresent()) {
			ChassisSpeeds speeds = driveSignalOpt.get();
			request = autonRequest.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond)
					.withRotationalRate(speeds.omegaRadiansPerSecond);
		} else
			request = swerveRequest;

		this.setControl(request);
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("/DriveTrain/BatteryVoltage", RobotController.getBatteryVoltage());

	}
}
