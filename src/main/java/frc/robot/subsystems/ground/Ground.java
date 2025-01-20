package frc.robot.subsystems.ground;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.ground.GroundConstants.kP;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Ground extends SubsystemBase {

  // In rotations
  private static final LoggedTunableNumber loweredAngle =
      new LoggedTunableNumber("Ground/Angles/lowered", 19);
  private static final LoggedTunableNumber raisedAngle =
      new LoggedTunableNumber("Ground/Angles/raised", 86);
  // In percent output
  private static final LoggedTunableNumber rollersSpeed =
      new LoggedTunableNumber("Ground/Speeds/groundRollers", 1);

  private Angle positionSetpoint = Degrees.zero();

  private final GroundIO io;
  private final GroundIOInputsAutoLogged inputs = new GroundIOInputsAutoLogged();

  public Ground(GroundIO io) {
    this.io = io;

    io.setPID(kP.get());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Ground", inputs);

    // Update controllers
    LoggedTunableNumber.ifChanged(hashCode(), () -> io.setPID(kP.get()), kP);

    // Logs
    Logger.recordOutput("Ground/positionSetpointRadians", positionSetpoint.in(Radians));
    Util.logSubsystem(this, "Ground");
  }

  private void lower() {
    positionSetpoint = Degrees.of(loweredAngle.get());
    io.setPivotPosition(positionSetpoint);
  }

  private void raise() {
    positionSetpoint = Degrees.of(raisedAngle.get());
    io.setPivotPosition(positionSetpoint);
  }

  private void rollersIn() {
    io.runGroundRollers(rollersSpeed.get());
  }

  private void rollersOut() {
    io.runGroundRollers(-rollersSpeed.get());
  }

  private void rollersStop() {
    io.runGroundRollers(0);
  }

  public Command raiseAndStopCmd() {
    return run(() -> {
          raise();
          rollersStop();
        })
        .withName("raise and stop");
  }

  public Command intakeCmd() {
    return run(() -> {
          lower();
          rollersIn();
        })
        .withName("ground");
  }

  public Command poopCmd() {
    return run(() -> {
          raise();
          rollersOut();
        })
        .withName("poop");
  }

  // In percent output

}
