package frc.robot.subsystems.rollers;

import frc.robot.util.LoggedTunableNumber;

public class RollersConstants {
  public static final int rollerMotorID = 0;

  public static final double intakingSpeed = 1;
  public static final double placeSpeed = 1;
  public static final double dealgifyingSpeed = 1;
  public static final double processorSpeed = -1;

  public static final LoggedTunableNumber algaeVelocityThreshold;
  public static final LoggedTunableNumber algaeCurrentThreshold;

  public static final int beamBreakNumber = 0;

  static {
    algaeVelocityThreshold = new LoggedTunableNumber("Reef/Rollers/algaeVelocityThreshold");
    algaeCurrentThreshold = new LoggedTunableNumber("Reef/Rollers/algaeCurrentThreshold");
  }
}
