package frc.robot.subsystems.carriage;

import frc.robot.util.LoggedTunableNumber;

public class CarriageConstants {
  public static final int rollerMotorID = 0;
  public static final int beamBreakNumber = 0;

  // TODO put in values velocity and current thresholds based off example
  public static final LoggedTunableNumber algaeVelocityThreshold =
      new LoggedTunableNumber("Carriage/algaeVelocityThreshold");
  public static final LoggedTunableNumber algaeCurrentThreshold =
      new LoggedTunableNumber("Carriage/algaeCurrentThreshold");

  public static final LoggedTunableNumber intakingSpeedVolts =
      new LoggedTunableNumber("Carriage/intakingSpeedVolts", 10);
  public static final LoggedTunableNumber placeSpeedVolts =
      new LoggedTunableNumber("Carriage/placeSpeedVolts", 10);
  public static final LoggedTunableNumber dealgifyingSpeedVolts =
      new LoggedTunableNumber("Carriage/dealgifyingSpeedVolts", 10);
  public static final LoggedTunableNumber processorSpeedVolts =
      new LoggedTunableNumber("Carriage/processorSpeedVolts", -10);

}
