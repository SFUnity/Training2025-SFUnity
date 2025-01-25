package frc.robot.subsystems.carriage;

import frc.robot.util.LoggedTunableNumber;

public class CarriageConstants {
  public static final int rollerMotorID = 0;

  public static final int currentLimit = 0;

  // Unit: volts
  public static final LoggedTunableNumber intakingSpeed;
  public static final LoggedTunableNumber placeSpeed;
  public static final LoggedTunableNumber dealgifyingSpeed;
  public static final LoggedTunableNumber processorSpeed;

  public static final LoggedTunableNumber algaeVelocityThreshold;
  public static final LoggedTunableNumber algaeCurrentThreshold;

  public static final int beamBreakNumber = 0;

  static {
    algaeVelocityThreshold = new LoggedTunableNumber("Carriage/algaeVelocityThreshold");
    algaeCurrentThreshold = new LoggedTunableNumber("Carriage/algaeCurrentThreshold");

    intakingSpeed = new LoggedTunableNumber("Carriage/intakingSpeed", 1);
    placeSpeed = new LoggedTunableNumber("Carriage/placeSpeed", 1);
    dealgifyingSpeed = new LoggedTunableNumber("Carriage/dealgifyingSpeed", 1);
    processorSpeed = new LoggedTunableNumber("Carriage/processorSpeed", -1);
  }
}
