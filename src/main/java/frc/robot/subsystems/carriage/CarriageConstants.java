package frc.robot.subsystems.carriage;

import frc.robot.util.LoggedTunableNumber;

public class CarriageConstants {
  public static final int rollerMotorID = 0;

  public static final int currentLimit = 0;

  // Unit: volts
  public static final LoggedTunableNumber intakingSpeedVolts;
  public static final LoggedTunableNumber placeSpeedVolts;
  public static final LoggedTunableNumber dealgifyingSpeedVolts;
  public static final LoggedTunableNumber processorSpeedVolts;

  public static final LoggedTunableNumber algaeVelocityThreshold;
  public static final LoggedTunableNumber algaeCurrentThreshold;

  public static final int beamBreakNumber = 0;

  static {
    algaeVelocityThreshold = new LoggedTunableNumber("Carriage/algaeVelocityThreshold");
    algaeCurrentThreshold = new LoggedTunableNumber("Carriage/algaeCurrentThreshold");

    intakingSpeedVolts = new LoggedTunableNumber("Carriage/intakingSpeedVolts", 10);
    placeSpeedVolts = new LoggedTunableNumber("Carriage/placeSpeedVolts", 10);
    dealgifyingSpeedVolts = new LoggedTunableNumber("Carriage/dealgifyingSpeedVolts", 10);
    processorSpeedVolts = new LoggedTunableNumber("Carriage/processorSpeedVolts", -10);
  }
}
