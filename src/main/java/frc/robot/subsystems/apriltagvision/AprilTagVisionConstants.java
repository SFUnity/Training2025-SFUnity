package frc.robot.subsystems.apriltagvision;

public class AprilTagVisionConstants {
  public static final double fieldBorderMargin = 0.5;

  // TODO everything below this needs updating
  public static final String reefName = "limelight-reef";
  // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
  public static final double reefForwardOffset = 0; // meters
  public static final double reefSideOffset = 0; // meters
  public static final double reefHeightOffset = 0; // meters
  public static final double reefRoll = 0; // degrees
  public static final double reefPitch = 0; // degrees
  public static final double reefYaw = 0; // degrees
  public static final double[] reefPosition = {
    reefForwardOffset, reefSideOffset, reefHeightOffset, reefRoll, reefPitch, reefYaw
  };
  public static final String sourceName = "limelight-source";
  // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
  public static final double sourceForwardOffset = 0; // meters
  public static final double sourceSideOffset = 0; // meters
  public static final double sourceHeightOffset = 0; // meters
  public static final double sourceRoll = 0; // degrees
  public static final double sourcePitch = 0; // degrees
  public static final double sourceYaw = 0; // degrees
  public static final double[] sourcePosition = {
    sourceForwardOffset, sourceSideOffset, sourceHeightOffset, sourceRoll, sourcePitch, sourceYaw
  };

  public static enum Pipelines {
    // TODO Add more pipelines
    APRILTAGHM;

    public static int getIndexFor(Pipelines pipeline) {
      switch (pipeline) {
        case APRILTAGHM:
          return 0;
        default:
          return 0;
      }
    }
  }
}
