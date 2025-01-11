// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {
  /** Flips an x coordinate to the correct side of the field based on the current alliance color. */
  public static double applyX(double xCoordinate) {
    if (shouldFlip()) {
      return ChoreoAllianceFlipUtil.flipX(xCoordinate);
    } else {
      return xCoordinate;
    }
  }

  /** Flips an y coordinate to the correct side of the field based on the current alliance color. */
  public static double applyY(double yCoordinate) {
    if (shouldFlip()) {
      return ChoreoAllianceFlipUtil.flipY(yCoordinate);
    } else {
      return yCoordinate;
    }
  }

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return ChoreoAllianceFlipUtil.flip(translation);
    } else {
      return translation;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return ChoreoAllianceFlipUtil.flip(rotation);
    } else {
      return rotation;
    }
  }

  /** Flips an angle based on the current alliance color. */
  public static Angle apply(Angle angle) {
    if (shouldFlip()) {
      return ChoreoAllianceFlipUtil.flip(new Rotation2d(angle)).getMeasure();
    } else {
      return angle;
    }
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return ChoreoAllianceFlipUtil.flip(pose);
    } else {
      return pose;
    }
  }

  public static Translation3d apply(Translation3d translation3d) {
    if (shouldFlip()) {
      return ChoreoAllianceFlipUtil.flip(translation3d);
    } else {
      return translation3d;
    }
  }

  public static boolean shouldFlip() {
    return ChoreoAllianceFlipUtil.shouldFlip();
  }
}
