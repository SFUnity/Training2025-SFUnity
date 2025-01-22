// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constantsGlobal;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public final class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.876);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line

  public static final Pose2d processorScore =
      new Pose2d(Units.inchesToMeters(235.726), .75 / 2, Rotation2d.fromDegrees(-90));

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  private static final double xOffset = Units.inchesToMeters(30.738) + .75 / 2 + .2;
  private static final double yOffset = Units.inchesToMeters(6.469);
  private static final Translation2d reefCenter =
      new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));

  public static enum Branch {
    A(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 1)))
            .transformBy(new Transform2d(xOffset, yOffset, new Rotation2d(Math.PI)))),
    B(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 1)))
            .transformBy(new Transform2d(xOffset, -yOffset, new Rotation2d(Math.PI)))),
    C(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 2)))
            .transformBy(new Transform2d(xOffset, yOffset, new Rotation2d(Math.PI)))),
    D(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 2)))
            .transformBy(new Transform2d(xOffset, -yOffset, new Rotation2d(Math.PI)))),
    E(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 3)))
            .transformBy(new Transform2d(xOffset, yOffset, new Rotation2d(Math.PI)))),
    F(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 3)))
            .transformBy(new Transform2d(xOffset, -yOffset, new Rotation2d(Math.PI)))),
    G(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 4)))
            .transformBy(new Transform2d(xOffset, yOffset, new Rotation2d(Math.PI)))),
    H(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 4)))
            .transformBy(new Transform2d(xOffset, -yOffset, new Rotation2d(Math.PI)))),
    I(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 5)))
            .transformBy(new Transform2d(xOffset, yOffset, new Rotation2d(Math.PI)))),
    J(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 5)))
            .transformBy(new Transform2d(xOffset, -yOffset, new Rotation2d(Math.PI)))),
    K(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 6)))
            .transformBy(new Transform2d(xOffset, yOffset, new Rotation2d(Math.PI)))),
    L(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 6)))
            .transformBy(new Transform2d(xOffset, -yOffset, new Rotation2d(Math.PI))));

    Branch(Pose2d pose) {
      this.pose = pose;
    }

    public final Pose2d pose;
  }

  public static enum Face {
    One(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180))
            .transformBy(new Transform2d(xOffset, 0, new Rotation2d(Math.PI))),
        Branch.A,
        Branch.B),
    Two(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 1)))
            .transformBy(new Transform2d(xOffset, 0, new Rotation2d(Math.PI))),
        Branch.C,
        Branch.D),
    Three(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 2)))
            .transformBy(new Transform2d(xOffset, 0, new Rotation2d(Math.PI))),
        Branch.E,
        Branch.F),
    Four(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 3)))
            .transformBy(new Transform2d(xOffset, 0, new Rotation2d(Math.PI))),
        Branch.G,
        Branch.H),
    Five(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 4)))
            .transformBy(new Transform2d(xOffset, 0, new Rotation2d(Math.PI))),
        Branch.I,
        Branch.J),
    Six(
        new Pose2d(reefCenter, Rotation2d.fromDegrees(180 - (60 * 5)))
            .transformBy(new Transform2d(xOffset, 0, new Rotation2d(Math.PI))),
        Branch.K,
        Branch.L);

    Face(Pose2d pose, Branch lefBranch, Branch rightBranch) {
      this.pose = pose;
      this.leftBranch = lefBranch;
      this.rightBranch = rightBranch;
    }

    public final Pose2d pose;
    public final Branch leftBranch;
    public final Branch rightBranch;
  }

  // TODO replace with robot heights instead
  public static enum ReefHeight {
    L3(Units.inchesToMeters(47.625)),
    L2(Units.inchesToMeters(31.875)),
    L1(Units.inchesToMeters(18)),
    AlgaeHigh(Units.inchesToMeters(55)),
    AlgaeLow(Units.inchesToMeters(40));

    ReefHeight(double height) {
      this.height = height;
    }

    public final double height;
  }
}
