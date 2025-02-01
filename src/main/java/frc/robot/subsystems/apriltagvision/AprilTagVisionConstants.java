package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.util.Units;

public class AprilTagVisionConstants {
  public static final double fieldBorderMargin = 0.5;

  // TODO this all needs testing to see if it's correct
  public static final String reefName = "limelight-reef";
  // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
  public static final double reefForwardOffset = Units.inchesToMeters(-6.3); // meters
  public static final double reefSideOffset = Units.inchesToMeters(-10); // meters
  public static final double reefHeightOffset = Units.inchesToMeters(12); // meters
  public static final double reefRoll = 0; // degrees
  public static final double reefPitch = 0; // degrees
  public static final double reefYaw = 0; // degrees
  public static final double[] reefPosition = {
    reefForwardOffset, reefSideOffset, reefHeightOffset, reefRoll, reefPitch, reefYaw
  };
  public static final String sourceName = "limelight-source";
  // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
  public static final double sourceForwardOffset = Units.inchesToMeters(-4.2); // meters
  public static final double sourceSideOffset = Units.inchesToMeters(0); // meters
  public static final double sourceHeightOffset = Units.inchesToMeters(39); // meters
  public static final double sourceRoll = 0; // degrees
  public static final double sourcePitch = 26; // degrees
  public static final double sourceYaw = 180; // degrees
  public static final double[] sourcePosition = {
    sourceForwardOffset, sourceSideOffset, sourceHeightOffset, sourceRoll, sourcePitch, sourceYaw
  };

  public static enum Pipelines {
    // TODO Add more pipelines
    HUMAN_MADE;

    public static int getIndexFor(Pipelines pipeline) {
      switch (pipeline) {
        case HUMAN_MADE:
          return 0;
        default:
          return 0;
      }
    }
  }
}
