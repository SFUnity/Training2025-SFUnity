package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class AprilTagVisionConstants {
  public static final double fieldBorderMargin = 0.2; // meters

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final String leftName = "limelight-left";
  // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
  public static final double leftForwardOffset = 0.29; // meters
  public static final double leftSideOffset = -0.194; // meters
  public static final double leftHeightOffset = 0.208; // meters
  public static final double leftRoll = 180; // degrees
  public static final double leftPitch = -20; // degrees
  public static final double leftYaw = -25; // degrees
  public static final double[] leftPosition = {
    leftForwardOffset, leftSideOffset, leftHeightOffset, leftRoll, leftPitch, leftYaw
  };
  public static final String rightName = "limelight-right";
  // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
  public static final double rightForwardOffset = 0.29; // meters
  public static final double rightSideOffset = 0.194; // meters
  public static final double rightHeightOffset = 0.208; // meters
  public static final double rightRoll = 180; // degrees
  public static final double rightPitch = -20; // degrees
  public static final double rightYaw = 25; // degrees
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
