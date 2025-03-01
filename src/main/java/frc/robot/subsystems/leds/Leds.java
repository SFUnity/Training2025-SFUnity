// Edited version of Mechanical Advantages 2024 Leds class

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj.LEDPattern.*;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.VirtualSubsystem;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Leds extends VirtualSubsystem {
  private static Leds instance;

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  // Robot state tracking
  public int loopCycleCount = 0;

  public boolean coralHeld = false;
  public boolean carriageAlgaeHeld = false;
  public boolean intakeAlgaeHeld = false;
  public boolean autoAllignActivated = false;
  public boolean intakingActivated = false;

  // Not in use
  public boolean alignedWithTarget = false;

  // Non-season Specific
  private Optional<Alliance> alliance = Optional.empty();
  private Color allianceColor = Color.kOrange;
  private Color secondaryDisabledColor = Color.kDarkBlue;

  private boolean lastEnabledAuto = false;
  public boolean autoFinished = false;
  public double autoFinishedTime = 0.0;
  private double lastEnabledTime = 0.0;

  public boolean lowBatteryAlert = false;
  private boolean estopped = false;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  // Constants
  private static final boolean prideLeds = false;
  private static final int minLoopCycleCount = 10;
  private static final int length = 150;
  // private static final Distance ledSpacing = Feet.of(8 / length);
  private static final Time breathDuration = Seconds.of(1);
  // private static final LinearVelocity waveFastCycleLength = InchesPerSecond.of(25.0);
  // private static final LinearVelocity waveAllianceCycleLength = InchesPerSecond.of(15.0);
  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal

  private LEDPattern pattern = solid(Color.kBlack); // Default to off

  private Leds() {
    leds = new AddressableLED(7);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                gradient(GradientType.kDiscontinuous, Color.kWhite, Color.kBlack)
                    .breathe(breathDuration)
                    .applyTo(buffer);
                ;
                leds.setData(buffer);
              }
            });
    loadingNotifier.startPeriodic(0.02);
  }

  public synchronized void periodic() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      allianceColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kDarkBlue : Color.kOrangeRed)
              .orElse(Color.kRed);
      secondaryDisabledColor = alliance.isPresent() ? Color.kWhite : Color.kBlue;
    }

    // Update auto state
    if (DriverStation.isDisabled()) {
      autoFinished = false;
    } else {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getFPGATimestamp();
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    // Select LED mode
    if (estopped) {
      pattern = solid(Color.kDarkRed);
    } else if (DriverStation.isDisabled()) {

      if (lastEnabledAuto && Timer.getFPGATimestamp() - lastEnabledTime < autoFadeMaxTime) {
        // Auto fade
        pattern =
            solid(Color.kGreen)
                .mask(
                    progressMaskLayer(
                        () -> 1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / autoFadeTime)));
      } else if (lowBatteryAlert) {
        pattern = solid(Color.kOrangeRed);
      } else if (prideLeds) {
        // Pride stripes
        pattern = rainbow(255, 128);
      } else {
        // Default pattern
        pattern = teamColors();
      }
    } else if (DriverStation.isAutonomous()) {
      pattern = gradient(GradientType.kContinuous, Color.kOrangeRed, Color.kDarkBlue);
      if (autoFinished) {
        pattern =
            solid(Color.kGreen)
                .mask(progressMaskLayer(() -> Timer.getFPGATimestamp() - autoFinishedTime));
      }
    } else { // Enabled
      if (autoAllignActivated) {
        blink(Color.kYellow, Seconds.of(0.75));
      }
      if (coralHeld) {
        pattern = solid(Color.kBrown);
      } else if (intakeAlgaeHeld) {
        pattern = solid(Color.kSeaGreen);
      } else if (carriageAlgaeHeld) {
        pattern = solid(Color.kPurple);
      } else {
        // No game piece state
        pattern = teamColors().atBrightness(Percent.of(0.5));
      }
    }

    // Update LEDs
    pattern.applyTo(buffer);
    leds.setData(buffer);

    // Logs
    Logger.recordOutput("LEDs/coralHeld", coralHeld);
    Logger.recordOutput("LEDs/carriageAlgaeHeld", carriageAlgaeHeld);
    Logger.recordOutput("LEDs/intakeAlgaeHeld", intakeAlgaeHeld);
    Logger.recordOutput("LEDs/autoFinished", autoFinished);
    Logger.recordOutput("LEDs/lowBatteryAlert", lowBatteryAlert);
    Logger.recordOutput("LEDs/autoAllignActivated", autoAllignActivated);
    Logger.recordOutput("LEDs/alignedWithTarget", alignedWithTarget);
    Logger.recordOutput("LEDs/lastEnabledAuto", lastEnabledAuto);
    Logger.recordOutput("LEDs/estopped", estopped);
  }

  private void blink(Color color, Time blink) {
    LEDPattern base = solid(color);
    pattern = base.blink(blink);
  }

  private LEDPattern teamColors() {
    return LEDPattern.steps(Map.of(0, new Color(255, 30, 0), 0.5, new Color(0, 0, 225)))
        .scrollAtRelativeSpeed(Percent.per(Second).of(35));
  }
}
