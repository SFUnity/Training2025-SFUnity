package frc.robot.subsystems.carriage;

import frc.robot.util.LoggedTunableNumber;

public class CarriageConstants {
  public static final int rollerMotorID = 0;

  public static final int currentLimit = 60;

  // TODO put in values velocity and current thresholds based off example
  public static final LoggedTunableNumber algaeVelocityThreshold =
      new LoggedTunableNumber("Carriage/algaeVelocityThreshold");
  public static final LoggedTunableNumber algaeCurrentThreshold =
      new LoggedTunableNumber("Carriage/algaeCurrentThreshold");

  public static final LoggedTunableNumber intakingSpeed =
      new LoggedTunableNumber("Carriage/intakingSpeed", 1);
  public static final LoggedTunableNumber placeSpeed =
      new LoggedTunableNumber("Carriage/placeSpeed", 1);
  public static final LoggedTunableNumber dealgifyingSpeed =
      new LoggedTunableNumber("Carriage/dealgifyingSpeed", 1);
  public static final LoggedTunableNumber processorSpeed =
      new LoggedTunableNumber("Carriage/processorSpeed", -1);

  public static final int beamBreakNumber = 0;
}
