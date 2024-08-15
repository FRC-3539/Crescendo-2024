package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.vision.photonvision.gtsam.GtsamInterface;
import frc.robot.vision.photonvision.gtsam.TagDetection;

import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends Thread {

	// Vision Variables
	AprilTagFieldLayout aprilTagFieldLayout;
	public GtsamInterface iface;

	public PhotonCamera backLeftCam;
	public static Transform3d robotToBackLeftCam = new Transform3d(new Translation3d(-0.3302, 0.2286, 0.53975),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-16.5), Math.toRadians(180)));

	public PhotonCamera backRightCam;
	public static Transform3d robotToBackRightCam = new Transform3d(new Translation3d(-0.3302, -0.2286, 0.53975),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-16.3), Math.toRadians(180)));

	public PhotonCamera frontNoteCam;
	public PhotonCamera backNoteCam;

	PhotonPoseEstimator backLeftPhotonPoseEstimator;
	PhotonPoseEstimator backRightPhotonPoseEstimator;

	PhotonPipelineResult lastBackLeftResult = new PhotonPipelineResult();
	PhotonPipelineResult lastBackRightResult = new PhotonPipelineResult();

	public VisionSubsystem() {
		super();
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

		this.iface = RobotContainer.drivetrainSubsystem.iface;
		sendTagLayout();
	}

	// Vision Methods

	public Optional<EstimatedRobotPose> getEstimatedBackLeftGlobalPose() {
		return backLeftPhotonPoseEstimator.update();
	}

	public Optional<EstimatedRobotPose> getEstimatedBackRightGlobalPose() {
		return backRightPhotonPoseEstimator.update();
	}

	public static void publishPose2d(String key, Pose2d pose) {
		SmartDashboard.putNumberArray(key, new double[]{pose.getTranslation().getX(), pose.getTranslation().getY(),
				pose.getRotation().getRadians()});
	}

	Alliance lastAlliance = null;

	public void log() {
		SmartDashboard.putBoolean("/Vision/BackLeft/Connected", backLeftCam.isConnected());
		SmartDashboard.putBoolean("/Vision/BackRight/Connected", backRightCam.isConnected());

		SmartDashboard.putBoolean("/Vision/FrontNoteCam/Connected", frontNoteCam.isConnected());
		SmartDashboard.putBoolean("/Vision/BackNoteCam/Connected", backNoteCam.isConnected());

	}

	public PhotonTrackedTarget getBestFrontNote() {
		var result = frontNoteCam.getLatestResult();
		var bestTarget = result.getBestTarget();
		return bestTarget;
	}
	public PhotonTrackedTarget getBestBackNote() {
		var result = backNoteCam.getLatestResult();
		var bestTarget = result.getBestTarget();
		return bestTarget;
	}

	public void sendTagLayout() {
		iface.sendLayout(aprilTagFieldLayout);
	}

	public void sendInitialGuess() {
		var val = getEstimatedBackLeftGlobalPose();
		var val2 = getEstimatedBackRightGlobalPose();
		if (val.isPresent()) {
			iface.sendGuess(RobotController.getFPGATime(), val.get().estimatedPose);
		} else if (val2.isPresent()) {
			iface.sendGuess(RobotController.getFPGATime(), val2.get().estimatedPose);
		} else {
			DriverStation.reportWarning("Sending Empty Pose3d", false);
			iface.sendGuess(RobotController.getFPGATime(), new Pose3d());
		}
		iface.setCamIntrinsics("BackLeft", backLeftCam.getCameraMatrix(), backLeftCam.getDistCoeffs());
		iface.setCamIntrinsics("BackRight", backRightCam.getCameraMatrix(), backRightCam.getDistCoeffs());
	}
	@Override
	public void run() {
		while (true) {
			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

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

			backLeftPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);
			backRightPhotonPoseEstimator.setFieldTags(aprilTagFieldLayout);

			var newLeft = backLeftCam.getLatestResult();
			var newRight = backRightCam.getLatestResult();

			if (newLeft.getTimestampSeconds() != lastBackLeftResult.getTimestampSeconds()) {
				lastBackLeftResult = newLeft;

				var tags = new ArrayList<TagDetection>();
				for (var result : newLeft.getTargets()) {
					tags.add(new TagDetection(result.getFiducialId(), result.getDetectedCorners()));
				}

				iface.sendVisionUpdate("BackLeft", (long) (newLeft.getTimestampSeconds() * 1e6), tags,
						VisionSubsystem.robotToBackLeftCam);
			}

			if (newRight.getTimestampSeconds() != lastBackRightResult.getTimestampSeconds()) {
				lastBackRightResult = newRight;

				var tags = new ArrayList<TagDetection>();
				for (var result : newRight.getTargets()) {
					tags.add(new TagDetection(result.getFiducialId(), result.getDetectedCorners()));
				}

				iface.sendVisionUpdate("BackRight", (long) (newRight.getTimestampSeconds() * 1e6), tags,
						VisionSubsystem.robotToBackRightCam);
			}
		}
	}
}
