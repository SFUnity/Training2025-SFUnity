package frc.robot.subsystems.carrage;

import frc.robot.util.LoggedTunableNumber;

public class CarrageConstants {
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
    algaeVelocityThreshold = new LoggedTunableNumber("Reef/Rollers/algaeVelocityThreshold");
    algaeCurrentThreshold = new LoggedTunableNumber("Reef/Rollers/algaeCurrentThreshold");

    intakingSpeed = new LoggedTunableNumber("Reef/Rollers/intakingSpeed", 1);
    placeSpeed = new LoggedTunableNumber("Reef/Rollers/placeSpeed", 1);
    dealgifyingSpeed = new LoggedTunableNumber("Reef/Rollers/dealgifyingSpeed", 1);
    processorSpeed = new LoggedTunableNumber("Reef/Rollers/processorSpeed", -1);
  }
}
