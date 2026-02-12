package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.lang.reflect.Array;
import java.security.Key;
import java.util.Calendar;
import java.util.Date;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.Random;
import java.util.concurrent.TimeUnit;

public class LEDSubsystem2 extends SubsystemBase {
  private static final int PWM_PORT = 9;
  private static final int NUMBER_OF_LEDS = 100;

  private static Color targetingColor = Color.kWhite;
  private boolean targetingBlink = false;

  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;
 
  // private final AddressableLEDBufferView buff;
  //Green and Red are flipped on the strip. The colors are swapped on purpose;
  private final Color RedColor = new Color(0, 255, 0);
  private final Color GreenColor = new Color(1, 255, 1);
  private final Color BlueColor = new Color(0, 1, 225);
  private final Color YellowColor = new Color(150, 150, 0);
  private final Color purpleColor = new Color(0, 150, 150);
  public LEDSubsystem2() {

    led = new AddressableLED(PWM_PORT);
    led.setLength(NUMBER_OF_LEDS);

    ledBuffer = new AddressableLEDBuffer(NUMBER_OF_LEDS);

    led.setData(ledBuffer);
    led.start();

    // Set the default command to turn the strip off, otherwise the last colors
    // written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    //setDefaultCommand(UpdatePatternCommand().withName("Off"));
  }
  

  // PUBLIC FUNCTIONS
  public void targetSearch() {
    targetingColor = (YellowColor);
    targetingBlink = false;

  }

  public void targetFound() {
    targetingColor = (YellowColor);
    targetingBlink = true;
  }
 
 
    public void targetLocked() {
    targetingColor = (RedColor);
    targetingBlink = false;
  }

  public void firingTurret() {
  targetingColor = (purpleColor);
  targetingBlink = false;
  }


  @Override
  public void periodic() {
    SmartDashboard.putString("Run Periodic", "running");
    // Periodically send the latest LED color data to the LED strip for it to
    // display
     updatePattern();
    led.setData(ledBuffer);
  }
// Any change of LED pattern should happen in function below
   public void updatePattern() {
    SmartDashboard.putString("Pattern", targetingColor.toString());
   
      LEDPattern ledBase = LEDPattern.solid(targetingColor);
    if(targetingBlink){
      ledBase = ledBase.blink(Seconds.of(0.25), Seconds.of(0.25));
    }
    ledBase.applyTo(ledBuffer);
  
  }


    /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command UpdatePatternCommand() {
    SmartDashboard.putString("Pattern", targetingColor.toString());
    return run(() -> updatePattern());
   // LEDPattern.targetingBlink(1,1).applyTo(ledBuffer);
  }
  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command TargetSearchCommand() {
    SmartDashboard.putString("y", "yes");
      return runOnce(() -> {targetSearch(); });
  }
  public Command TargetLockedCommand() {
    SmartDashboard.putString("b", "nyes");
      return runOnce(() -> {targetLocked(); });
  }
public Command TargetFoundCommand() {
  SmartDashboard.putString("x", "chyes");
    return runOnce(() -> {targetFound(); });
  }
  public Command FiringTurretCommand() {
    SmartDashboard.putString("a", "yess");
    return runOnce(() -> {firingTurret(); });
  }
}
