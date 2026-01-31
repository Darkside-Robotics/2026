package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.lang.reflect.Array;
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
public class LEDSubsystem2 extends SubsystemBase {
  //private static final int PWM_PORT = 9;
  //private static final int NUMBER_OF_LEDS = 65;

   private static final int kPort = 9;
  private static final int kLength = 100;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private final AddressableLEDBufferView buff;

  public LEDSubsystem2() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();
    buff = m_buffer.createView(0, 10);
    LEDPattern base = LEDPattern.solid(Color.kBlack);
    base.applyTo(buff);
    Color[] myarray = new Color[] {Color.kRed, Color.kYellow, Color.kBlue};
    for (var i = 0; i > 10; i++) {
      Random rand = new Random(); 
     int n1 = rand.nextInt(2);
     System.out.println(n1);
     // LEDPattern myLedPattern = LEDPattern.solid(myarray[n1]);
     buff.setLED(i, myarray[n1]);
    }

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    // setDefaultCommand(runPattern(LEDPattern.solid(Color.kPink)).withName("Off"));
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    m_led.setData(m_buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  
  }
}
