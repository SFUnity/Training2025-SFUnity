package frc.robot.subsystems.apriltagvision;

public class AprilTagVisionConstants {
  public static final double fieldBorderMargin = 0.5;

  // TODO everything below this needs updating
  public static final String reefName = "limelight-reef";
  // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
  public static final double reefForwardOffset = 0; //meters
  public static final double reefSideOffset = 0; //meters
  public static final double reefHeightOffset = 0; //meters
  public static final double reefRoll = 0; //degrees
  public static final double reefPitch = 0; //degrees
  public static final double reefYaw = 0; //degrees
  public static final double[] reefPosition = {reefForwardOffset, reefSideOffset, reefHeightOffset, reefRoll, reefPitch, reefYaw};
  public static final String sourceName = "limelight-source";
  public static final double limelight1ForwardOffset = 0; // meters
  public static final double limelight1SideOffset = 0; // meters
  public static final double limelight1HeightOffset = 0; // meters
  public static final double limelight1Roll = 0; // degrees
  public static final double limelight1Pitch = 0; // degrees
  public static final double limelight1Yaw = 0; // degrees
  public static final double[] limelight1Position = {
    limelight1ForwardOffset,
    limelight1SideOffset,
    limelight1HeightOffset,
    limelight1Roll,
    limelight1Pitch,
    limelight1Yaw
  };
  public static final String limelight2Name = "source";
  // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
  public static final double limelight2ForwardOffset = 0; // meters
  public static final double limelight2SideOffset = 0; // meters
  public static final double limelight2HeightOffset = 0; // meters
  public static final double limelight2Roll = 0; // degrees
  public static final double limelight2Pitch = 0; // degrees
  public static final double limelight2Yaw = 0; // degrees
  public static final double[] limelight2Position = {
    limelight2ForwardOffset,
    limelight2SideOffset,
    limelight2HeightOffset,
    limelight2Roll,
    limelight2Pitch,
    limelight2Yaw
  };

  public static enum Pipelines {
    BLUE_SPEAKER,
    RED_SPEAKER,
    SOURCE;

    public static int getIndexFor(Pipelines pipeline) {
      switch (pipeline) {
        case BLUE_SPEAKER:
          return 0;
        case RED_SPEAKER:
          return 1;
        case SOURCE:
          return 2;
        default:
          return 0;
      }
    }
  }
}
