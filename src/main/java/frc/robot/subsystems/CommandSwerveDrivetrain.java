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
import java.util.List;
import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.WPIUtilJNI;
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
import frc.robot.vision.photonvision.gtsam.GtsamInterface;
import frc.robot.vision.photonvision.gtsam.TagDetection;

import java.util.Arrays;
import org.frcteam3539.Byte_Swerve_Lib.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam3539.Byte_Swerve_Lib.control.PidConstants;
import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory;
import org.frcteam3539.Byte_Swerve_Lib.util.DrivetrainFeedforwardConstants;
import org.frcteam3539.Byte_Swerve_Lib.util.HolonomicFeedforward;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
	/** Creates a new DrivetrainSubsystem. */
	private final HolonomicMotionProfiledTrajectoryFollower follower;

	// Vision Variables
	AprilTagFieldLayout aprilTagFieldLayout;

	public boolean sentInitial = false;

	public static PhotonCamera backLeftCam;
	public static Transform3d robotToBackLeftCam = new Transform3d(new Translation3d(-0.3302, 0.2286, 0.53975),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-16.5), Math.toRadians(180)));

	public static PhotonCamera backRightCam;
	public static Transform3d robotToBackRightCam = new Transform3d(new Translation3d(-0.3302, -0.2286, 0.53975),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-16.3), Math.toRadians(180)));

	public static PhotonCamera frontNoteCam;
	public static PhotonCamera backNoteCam;

	PhotonPoseEstimator backLeftPhotonPoseEstimator;
	PhotonPoseEstimator backRightPhotonPoseEstimator;

	PhotonPipelineResult lastBackLeftResult = new PhotonPipelineResult();
	PhotonPipelineResult lastBackRightResult = new PhotonPipelineResult();

	public GtsamInterface iface;

	private SwerveRequest swerveRequest = new SwerveRequest.Idle();

	Translation2d blueSpeakerCoordinate = new Translation2d(0, 5.55);
	Translation2d redSpeakerCoordinate = new Translation2d(0, 2.67);

	public double velocityX = 0.0;
	public double velocityY = 0.0;

	public long lastOdomUpdate = 0;

	public double maxVelocity = 0.0;
	public double maxRotationalVelocity = 0.0;

	public Pigeon2 pigeon = new Pigeon2(IDConstants.PigeonID, "canivore");

	Alliance lastAlliance;

	long sentTime = 0;

	public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
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

		this.iface = new GtsamInterface(List.of("BackLeft", "BackRight"));

		// this.registerTelemetry(this::updateGtSam);

		try {
			aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

		} catch (Exception e) {
			System.out.println("ERROR Loading April Tag DATA");
			aprilTagFieldLayout = null;
		}

		backLeftCam = new PhotonCamera("BackLeft");
		backLeftPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backLeftCam, robotToBackLeftCam);

		backRightCam = new PhotonCamera("BackRight");
		backRightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backRightCam, robotToBackRightCam);

		frontNoteCam = new PhotonCamera("FrontNoteCam");
		backNoteCam = new PhotonCamera("BackNoteCam");

		backLeftCam.setVersionCheckEnabled(false);
		backRightCam.setVersionCheckEnabled(false);
		backNoteCam.setVersionCheckEnabled(false);
		frontNoteCam.setVersionCheckEnabled(false);

		sendTagLayout();

	}
	public void sendTagLayout() {
		iface.sendLayout(aprilTagFieldLayout);
	}

	public HolonomicMotionProfiledTrajectoryFollower getFollower() {
		return follower;
	}

	public void seedFieldRelative(Trajectory trajectory) {
		this.seedFieldRelative(trajectory.calculate(0).getPathState().getPose2d());
		SmartDashboard.putNumber("MEEEE", WPIUtilJNI.now());
		this.iface.sendGuess(WPIUtilJNI.now(), new Pose3d(trajectory.calculate(0).getPathState().getPose2d()));
	}

	public Pose2d getPose2d() {
		return this.iface.getLatencyCompensatedPoseEstimate().toPose2d();
		// return m_odometry.getEstimatedPosition();
	}

	public void applyRequest(SwerveRequest request) {
		this.swerveRequest = request;
	}

	public void cancelPath() {
		follower.cancel();
	}

	public void followPath(Trajectory t) {
		follower.follow(t);
	}

	public Rotation2d getRobotRoll() {
		return Rotation2d.fromDegrees(
				BaseStatusSignal.getLatencyCompensatedValue(pigeon.getRoll(), pigeon.getAngularVelocityYDevice()));
	}
	public Translation2d getOffsetTarget() {
		Translation2d offsetTarget;
		double noteSpeed = 16;
		double distanceToTarget = 0;
		if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
			distanceToTarget = Math.sqrt(Math.pow(
					RobotContainer.drivetrainSubsystem.getPose2d().getTranslation().getDistance(redSpeakerCoordinate),
					2) + Math.pow(2.05, 2));
			double timeToTarget = distanceToTarget / noteSpeed;
			offsetTarget = new Translation2d(redSpeakerCoordinate.getX() - velocityX * timeToTarget,
					redSpeakerCoordinate.getY() - velocityY * timeToTarget);
		} else {
			distanceToTarget = Math.sqrt(Math.pow(
					RobotContainer.drivetrainSubsystem.getPose2d().getTranslation().getDistance(blueSpeakerCoordinate),
					2) + Math.pow(2.05, 2));
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
	public void updateGtSam(SwerveDriveState state) {

	}
	SwerveModulePosition[] m_last_modulePositions;
	int test = 0;

	public void sendInitialGuess() {

		long time = RobotController.getFPGATime();

		DriverStation.reportWarning("Sending Empty Pose3d", false);
		iface.sendGuess(time, new Pose3d(new Translation3d(3, 2, 0), new Rotation3d()));

		sentInitial = true;

		sentTime = time;

	}
	@Override
	public void periodic() {
		var now = RobotController.getFPGATime();
		lastOdomUpdate = now;
		SmartDashboard.putNumber("FPGA Time", now);
		if (m_last_modulePositions == null) {
			this.m_last_modulePositions = new SwerveModulePosition[m_modulePositions.length];
			for (int i = 0; i < m_modulePositions.length; i++) {
				this.m_last_modulePositions[i] = m_modulePositions[i].copy();
			}
		}

		var twist = m_kinematics.toTwist2d(new SwerveDriveWheelPositions(m_last_modulePositions),
				new SwerveDriveWheelPositions(m_modulePositions));
		// System.out.println(twist.dx);

		this.m_last_modulePositions = new SwerveModulePosition[m_modulePositions.length];
		for (int i = 0; i < m_modulePositions.length; i++) {
			this.m_last_modulePositions[i] = m_modulePositions[i].copy();
		}

		var twist3 = new Twist3d(twist.dx, twist.dy, 0, 0, 0, twist.dtheta);

		iface.sendOdomUpdate(now, twist3);

		velocityX = ChassisSpeeds.fromRobotRelativeSpeeds(this.getState().speeds,
				this.getPose2d().getRotation()).vxMetersPerSecond;
		velocityY = ChassisSpeeds.fromRobotRelativeSpeeds(this.getState().speeds,
				this.getPose2d().getRotation()).vyMetersPerSecond;

		SwerveRequest request = new SwerveRequest.Idle();

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
			request = new SwerveRequest.RobotCentric().withVelocityX(speeds.vxMetersPerSecond)
					.withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond);
		} else
			request = swerveRequest;

		this.setControl(request);
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("/DriveTrain/BatteryVoltage", RobotController.getBatteryVoltage());

		if (DriverStation.getAlliance().isPresent()) {
			if (DriverStation.getAlliance().get() == Alliance.Blue && lastAlliance != Alliance.Blue) {
				lastAlliance = Alliance.Blue;
				aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
			}
			if (DriverStation.getAlliance().get() == Alliance.Red && lastAlliance != Alliance.Red) {

				lastAlliance = Alliance.Red;
				aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
			}
		}

		backRightPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);

		var newRight = backRightCam.getLatestResult();

		var tags = new ArrayList<TagDetection>();
		for (var result : newRight.getTargets()) {
			tags.add(new TagDetection(result.getFiducialId(), result.getDetectedCorners()));
		}
		if (sentInitial && sentTime < now - 3000000) {
			iface.sendVisionUpdate("BackRight", (long) (newRight.getTimestampSeconds() * 1000000), tags,
					VisionSubsystem.robotToBackRightCam);

		}

	}
}
