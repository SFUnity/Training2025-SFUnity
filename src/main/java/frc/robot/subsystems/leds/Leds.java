// Edited version of Mechanical Advantages 2024 Leds class

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.wpilibj.LEDPattern.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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

public class Leds extends VirtualSubsystem {
  private static Leds instance;

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  // Robot state tracking
  // TODO change these to be what you need for the season
  public int loopCycleCount = 0;
  public boolean intakeWorking = true;
  
  public boolean coralHeld = false;
  public boolean carriageAlgaeHeld = false;
  public boolean intakeAlgaeHeld = false;
  public boolean tagsDetected = false;
  public boolean autoFinished = false;
  public double autoFinishedTime = 0.0;
  public boolean lowBatteryAlert = false;
  public boolean autoAllignActivated = false;
  public boolean alignedWithTarget = false;


  private Optional<Alliance> alliance = Optional.empty();
  private Color allianceColor = Color.kOrange;
  private Color secondaryDisabledColor = Color.kDarkBlue;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;

  // Constants
  private static final boolean prideLeds = false;
  private static final int minLoopCycleCount = 10;
  private static final int length = 150;
  private static final Distance ledSpacing = Meters.of(1.0 / 60); // TODO get from specs sheet
  private static final Time breathDuration = Seconds.of(1);
  private static final LinearVelocity waveFastCycleLength = InchesPerSecond.of(25.0);
  private static final LinearVelocity waveAllianceCycleLength = InchesPerSecond.of(15.0);
  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal

  private LEDPattern pattern = solid(Color.kBlack); // Default to off

  // ! May have lost something when copying into our code
  private Leds() {
    leds = new AddressableLED(9);
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
        rainbow(255, 128).scrollAtAbsoluteSpeed(InchesPerSecond.of(1), ledSpacing);
      } else {
        // Default pattern
        pattern =
            gradient(GradientType.kContinuous, allianceColor, secondaryDisabledColor)
                .scrollAtAbsoluteSpeed(waveAllianceCycleLength, ledSpacing);
      }
    } else if (DriverStation.isAutonomous()) {
      pattern =
          gradient(GradientType.kContinuous, Color.kOrangeRed, Color.kDarkBlue)
              .scrollAtAbsoluteSpeed(waveFastCycleLength, ledSpacing);
      if (autoFinished) {
        pattern =
            solid(Color.kGreen)
                .mask(progressMaskLayer(() -> Timer.getFPGATimestamp() - autoFinishedTime));
      }
    } else { // Enabled
      if(alignedWithTarget){
        pattern = solid(Color.kGreen);
      }
      else if(autoAllignActivated){
        blink(Color.kGreen, Seconds.of(0.5));
      }
      if(coralHeld){
        pattern = solid(Color.kWhite);
      }
      else if(intakeAlgaeHeld){
        pattern = solid(Color.kSeaGreen);
      }
      else if(carriageAlgaeHeld){
        pattern = solid(Color.kPurple);
      }
      else{
        teamColors();
      }

      
      // if (!coralHeld) {
      //   if (intakeWorking) {
      //     pattern = solid(Color.kRed);
      //   } else {
      //     pattern = solid(Color.kYellow);
      //   }
      // } else if (!tagsDetected) {
      //   pattern = solid(Color.kBlue);
      // } else if (!alignedWithTarget) {
      //   pattern = solid(Color.kPurple);
      // } else if (alignedWithTarget) {
      //   pattern = solid(Color.kGreen);
      // }
    }

    // Update LEDs
    pattern.applyTo(buffer);
    leds.setData(buffer);
  }


  private void blink(Color color, Time blink){
    LEDPattern base = solid(color);
    pattern =  base.blink(blink);
  }

  private void teamColors(){
    LEDPattern base = LEDPattern.steps(Map.of(0, Color.kOrange, 0.5, Color.kBlue));
    pattern = base.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), ledSpacing);
  }
}
