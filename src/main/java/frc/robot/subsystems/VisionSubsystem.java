package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends Thread {

	// Vision Variables
	AprilTagFieldLayout aprilTagFieldLayout;

	public PhotonCamera backLeftCam;
	Transform3d robotToBackLeftCam = new Transform3d(new Translation3d(-0.3302, 0.2286, 0.53975),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-16.5), Math.toRadians(180)));

	public PhotonCamera backRightCam;
	Transform3d robotToBackRightCam = new Transform3d(new Translation3d(-0.3302, -0.2286, 0.53975),
			new Rotation3d(Math.toRadians(0), Math.toRadians(-16.3), Math.toRadians(180)));

	public PhotonCamera frontNoteCam;
	public PhotonCamera backNoteCam;

	PhotonPoseEstimator backLeftPhotonPoseEstimator;
	PhotonPoseEstimator backRightPhotonPoseEstimator;

	Optional<EstimatedRobotPose> resultBackLeft;
	Optional<EstimatedRobotPose> resultBackRight;
	boolean useVision = true;
	double backLeftLastTimeStamp = 0;
	double backRightLastTimeStamp = 0;

	double visionRatio = 10;

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
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBackLeftCam);

		backRightCam = new PhotonCamera("BackRight");
		backRightPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBackRightCam);

		frontNoteCam = new PhotonCamera("FrontNoteCam");
		backNoteCam = new PhotonCamera("BackNoteCam");

		setVisionWeights(.2, .2, 10);
	}

	// Vision Methods

	public Optional<EstimatedRobotPose> getEstimatedBackLeftGlobalPose() {
		Optional<EstimatedRobotPose> visionEst = Optional.empty();
		for (var change : backLeftCam.getAllUnreadResults()) {
			visionEst = backLeftPhotonPoseEstimator.update(change);
		}
		return visionEst;
	}

	public Optional<EstimatedRobotPose> getEstimatedBackRightGlobalPose() {
		Optional<EstimatedRobotPose> visionEst = Optional.empty();
		for (var change : backRightCam.getAllUnreadResults()) {
			visionEst = backRightPhotonPoseEstimator.update(change);
		}
		return visionEst;
	}

	public void useVision(boolean useVision) {
		this.useVision = useVision;
	}

	public void setVisionWeights(double visionX, double visionY, double visionDeg) {
		RobotContainer.drivetrainSubsystem
				.setVisionMeasurementStdDevs(VecBuilder.fill(visionX, visionY, Units.degreesToRadians(visionDeg)));
	}

	public static void publishPose2d(String key, Pose2d pose) {
		SmartDashboard.putNumberArray(key, new double[]{pose.getTranslation().getX(), pose.getTranslation().getY(),
				pose.getRotation().getRadians()});
	}

	Alliance lastAlliance = null;
	public void addVisionMeasurement(Pose2d pose, double timestampSeconds, Matrix<N3, N1> weights) {
		RobotContainer.drivetrainSubsystem.addVisionMeasurement(pose, timestampSeconds, weights);
	}

	public void log() {
		SmartDashboard.putBoolean("/Vision/BackLeft/Connected", backLeftCam.isConnected());
		SmartDashboard.putBoolean("/Vision/BackRight/Connected", backRightCam.isConnected());

		SmartDashboard.putBoolean("/Vision/FrontNoteCam/Connected", frontNoteCam.isConnected());
		SmartDashboard.putBoolean("/Vision/BackNoteCam/Connected", backNoteCam.isConnected());

	}
	public Matrix<N3, N1> getVisionWeights(double distanceRatio, int numTargets) {
		double targetMultiplier = 1;
		double visionCutOffDistance = 4;
		distanceRatio = 0.1466 * Math.pow(1.6903, distanceRatio);
		if (numTargets == 1) {
			if (distanceRatio > visionCutOffDistance) {
				return new Matrix<N3, N1>(new SimpleMatrix(new double[]{99999, 99999, 99999}));
			}
			targetMultiplier = 3;
		}
		Matrix<N3, N1> weights = new Matrix<N3, N1>(new SimpleMatrix(new double[]{distanceRatio * targetMultiplier,
				distanceRatio * targetMultiplier, 3 + 15 * distanceRatio * targetMultiplier}));
		return weights;
	}
	public PhotonTrackedTarget getBestFrontNote() {
		var results = frontNoteCam.getAllUnreadResults();
		var result = results.get(results.size() - 1);
		var bestTarget = result.getBestTarget();
		return bestTarget;
	}
	public PhotonTrackedTarget getBestBackNote() {
		var results = backNoteCam.getAllUnreadResults();
		var result = results.get(results.size() - 1);
		var bestTarget = result.getBestTarget();
		return bestTarget;
	}
	@Override
	public void run() {
		/* Run as fast as possible, our signals will control the timing */
		while (true) {
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			// Vision Calculations
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

			this.resultBackLeft = getEstimatedBackLeftGlobalPose();
			this.resultBackRight = getEstimatedBackRightGlobalPose();

			if (useVision) {

				if (resultBackLeft.isPresent()) {
					EstimatedRobotPose camPoseBackLeft = resultBackLeft.get();
					double backLeftTimeStamp = camPoseBackLeft.timestampSeconds;
					if (backLeftTimeStamp > Timer.getFPGATimestamp()) {
						backLeftTimeStamp = Timer.getFPGATimestamp();
					}

					double sum = 0;
					for (PhotonTrackedTarget target : camPoseBackLeft.targetsUsed) {
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						sum += resultBackLeft.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					sum /= camPoseBackLeft.targetsUsed.size();
					double distanceRatio = sum;
					Matrix<N3, N1> weights = getVisionWeights(distanceRatio, camPoseBackLeft.targetsUsed.size());

					if (backLeftTimeStamp != backLeftLastTimeStamp) {
						publishPose2d("/DriveTrain/BackLeftCamPose", camPoseBackLeft.estimatedPose.toPose2d());
						SmartDashboard.putString("/Vision/BackLeftWeights", weights.toString());
						RobotContainer.drivetrainSubsystem.addVisionMeasurement(
								camPoseBackLeft.estimatedPose.toPose2d(), backLeftTimeStamp, weights);

					}
					backLeftLastTimeStamp = backLeftTimeStamp;
				}

				if (resultBackRight.isPresent()) {
					EstimatedRobotPose camPoseBackRight = resultBackRight.get();
					double backRightTimeStamp = camPoseBackRight.timestampSeconds;

					if (backRightTimeStamp > Timer.getFPGATimestamp()) {
						backRightTimeStamp = Timer.getFPGATimestamp();
					}

					double sum = 0;
					for (PhotonTrackedTarget target : camPoseBackRight.targetsUsed) {
						Translation2d tagPosition = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get()
								.getTranslation().toTranslation2d();
						sum += resultBackRight.get().estimatedPose.toPose2d().getTranslation().getDistance(tagPosition);
					}
					sum /= camPoseBackRight.targetsUsed.size();
					double distanceRatio = sum;
					Matrix<N3, N1> weights = getVisionWeights(distanceRatio, camPoseBackRight.targetsUsed.size());

					if (backRightTimeStamp != backRightLastTimeStamp) {
						publishPose2d("/DriveTrain/BackRightCamPose", camPoseBackRight.estimatedPose.toPose2d());
						SmartDashboard.putString("/Vision/BackRightWeights", weights.toString());
						RobotContainer.drivetrainSubsystem.addVisionMeasurement(
								camPoseBackRight.estimatedPose.toPose2d(), backRightTimeStamp, weights);
					}
					backRightLastTimeStamp = backRightTimeStamp;
				}

			}
		}
	}
}
