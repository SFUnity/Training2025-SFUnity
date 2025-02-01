package frc.robot.util;

import static frc.robot.constantsGlobal.FieldConstants.*;
import static frc.robot.util.AllianceFlipUtil.apply;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;

public class PoseManager {
  static final Lock odometryLock = new ReentrantLock();
  public SwerveModulePosition[] lastModulePositions = // For reseting pose
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  public Rotation2d rawGyroRotation = new Rotation2d();
  private Twist2d robotVelocity = new Twist2d();

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public PoseManager() {}

  public void addOdometryMeasurementWithTimestamps(
      double currentTime, SwerveModulePosition[] modulePositions) {
    lastModulePositions = modulePositions;
    poseEstimator.updateWithTime(currentTime, rawGyroRotation, modulePositions);
  }

  public void addVisionMeasurement(Pose2d estimatedPose, double timestamp, Matrix<N3, N1> stdDevs) {
    // Add result because all checks passed
    poseEstimator.addVisionMeasurement(estimatedPose, timestamp, stdDevs);
  }

  public void addVelocityData(Twist2d robotVelocity) {
    this.robotVelocity = robotVelocity;
  }

  public double getDistanceTo(Pose2d pose) {
    return getDistanceTo(pose.getTranslation());
  }

  public double getDistanceTo(Translation3d translation) {
    return getDistanceTo(translation.toTranslation2d());
  }

  public double getDistanceTo(Translation2d translation) {
    Translation2d currentTranslation = getPose().getTranslation();
    return currentTranslation.getDistance(translation);
  }

  public Rotation2d getHorizontalAngleTo(Pose2d pose) {
    return getHorizontalAngleTo(pose.getTranslation());
  }

  public Rotation2d getHorizontalAngleTo(Translation3d translation) {
    return getHorizontalAngleTo(translation.toTranslation2d());
  }

  public Rotation2d getHorizontalAngleTo(Translation2d translation) {
    Translation2d currentTranslation = getPose().getTranslation();
    Rotation2d theta = translation.minus(currentTranslation).getAngle();
    return theta;
  }

  public Rotation2d getVerticalAngleTo(Translation3d translation) {
    double horizontalDiff = getDistanceTo(translation);
    double zDiff = translation.getZ();
    Rotation2d theta = new Rotation2d(Math.atan2(zDiff, horizontalDiff));
    return theta;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the current odometry translation. */
  public Translation2d getTranslation() {
    return getPose().getTranslation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, lastModulePositions, pose);
  }

  @AutoLogOutput(key = "Odometry/FieldVelocity")
  public Twist2d fieldVelocity() {
    Translation2d linearFieldVelocity =
        new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(getPose().getRotation());
    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
  }

  @AutoLogOutput(key = "Odometry/RobotVelocity")
  public Twist2d robotVelocity() {
    return robotVelocity;
  }

  public Face closestFace() {
    Face closestFace = Face.One;
    double distanceToClosestFace = Double.MAX_VALUE;
    for (Face face : Face.values()) {
      double distance = getDistanceTo(apply(face.getPose()));
      if (distance < distanceToClosestFace) {
        distanceToClosestFace = distance;
        closestFace = face;
      }
    }
    return closestFace;
  }

  public Branch closestLeftBranch() {
    Branch closestBranch = Face.One.leftBranch;
    double distanceToClosestLeftBranch = Double.MAX_VALUE;
    for (Face face : Face.values()) {
      double distance = getDistanceTo(apply(face.leftBranch.getPose()));
      if (distance < distanceToClosestLeftBranch) {
        distanceToClosestLeftBranch = distance;
        closestBranch = face.leftBranch;
      }
    }
    return closestBranch;
  }

  public Branch closestRightBranch() {
    Branch closestBranch = Face.One.rightBranch;
    double distanceToClosestRightBranch = Double.MAX_VALUE;
    for (Face face : Face.values()) {
      double distance = getDistanceTo(apply(face.rightBranch.getPose()));
      if (distance < distanceToClosestRightBranch) {
        distanceToClosestRightBranch = distance;
        closestBranch = face.rightBranch;
      }
    }
    return closestBranch;
  }

  public Pose2d closestStation() {
    final Pose2d leftFaceFlipped = apply(CoralStation.leftCenterFace);
    final Pose2d rightFaceFlipped = apply(CoralStation.rightCenterFace);

    if (getDistanceTo(leftFaceFlipped) < getDistanceTo(rightFaceFlipped)) {
      return leftFaceFlipped;
    } else {
      return rightFaceFlipped;
    }
  }

  @AutoLogOutput
  public double distanceToStationFace() {
    Pose2d station =
        closestStation()
            .transformBy(new Transform2d(intakeDistanceMeters.get(), 0, Rotation2d.kZero));
    Rotation2d angleToStation = getHorizontalAngleTo(station);
    Rotation2d stationAngle = station.getRotation();
    double hypotenuse = getDistanceTo(station);
    double angleDiff = angleToStation.minus(stationAngle).getRadians();
    return -Math.cos(angleDiff) * hypotenuse;
  }
}
