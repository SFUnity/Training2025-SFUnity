package frc.robot.subsystems.reef.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
  private final RollersIO io;
  private final RollersIOInputsAutoLogged inputs;

  public Rollers(RollersIO io, RollersIOInputsAutoLogged inputs) {
    this.io = io;
    this.inputs = inputs;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  private void runRollers(double volts) {
    io.runVolts(volts);
  }

  private void stopRollers() {
    io.stop();
  }

  public Command placeCoral() {
    return run(() -> {
          io.runMotorStraight();
          runRollers(RollersConstants.rollersPlaceSpeed);
        })
        .withName("placeCoralRollers");
  }

  public Command intakeCoral() {
    return run(() -> {
          io.runMotorStraight();
          runRollers(RollersConstants.rollersIntakingSpeed);
        })
        .withName("intakeCoralRollers");
  }

  public Command scoreCoral() {
    return run(() -> {
          io.reverseMotor();
          runRollers(RollersConstants.rollersIntakingSpeed);
        })
        .withName("scoreCoral");
  }
  // TODO: add "until when 2m distance sensor gets set up"
}
