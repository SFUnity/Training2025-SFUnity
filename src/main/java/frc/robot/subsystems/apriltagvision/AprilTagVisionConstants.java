package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.util.Units;

public class AprilTagVisionConstants {
  public static final double fieldBorderMargin = 0.5;

  // TODO this all needs testing to see if it's correct
  public static final String leftName = "limelight-left";
  // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
  public static final double leftForwardOffset = Units.inchesToMeters(11.285); // meters
  public static final double leftSideOffset = Units.inchesToMeters(7.683); // meters
  public static final double leftHeightOffset = Units.inchesToMeters(8.252); // meters
  public static final double leftRoll = 0; // degrees
  public static final double leftPitch = -25; // degrees
  public static final double leftYaw = 25; // degrees
  public static final double[] leftPosition = {
    leftForwardOffset, leftSideOffset, leftHeightOffset, leftRoll, leftPitch, leftYaw
  };
  public static final String rightName = "limelight-right";
  // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
  public static final double rightForwardOffset = Units.inchesToMeters(11.285); // meters
  public static final double rightSideOffset = Units.inchesToMeters(-7.683); // meters
  public static final double rightHeightOffset = Units.inchesToMeters(8.252); // meters
  public static final double rightRoll = 0; // degrees
  public static final double rightPitch = -25; // degrees
  public static final double rightYaw = -25; // degrees
  public static final double[] rightPosition = {
    rightForwardOffset, rightSideOffset, rightHeightOffset, rightRoll, rightPitch, rightYaw
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
