package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

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

public class LEDSubsystem extends SubsystemBase {
  private static final int kPort = 3;
  private static final int kLength = 45;

  private static int elevatorHeight = 0;
  private static Color elevatorColor = Color.kGreen;
  private final AddressableLEDBufferView[] elevatorBrackets;
  private final AddressableLEDBufferView utilitysection;
  
  private final AddressableLEDBufferView spacersection;  
  private final AddressableLEDBufferView spacersection2;


  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private Date startTime;
  private boolean reversed = false;

  private boolean elevatorBlink = false;

  public LEDSubsystem() {
    m_led = new AddressableLED(kPort);
    m_led.setLength(kLength);
    m_buffer = new AddressableLEDBuffer(kLength);


    elevatorBrackets = new AddressableLEDBufferView[4];
    elevatorBrackets[0] = m_buffer.createView(0, 7);
    elevatorBrackets[1] = m_buffer.createView(8, 15);
    elevatorBrackets[2] = m_buffer.createView(16, 23);
    elevatorBrackets[3] = m_buffer.createView(24, 31);

    utilitysection = m_buffer.createView(34, 43);
    spacersection = m_buffer.createView(32, 33);
    spacersection2 = m_buffer.createView(44, 44);

    LEDPattern pattern = LEDPattern.solid(Color.kBlack);
    pattern.applyTo(m_buffer);

    m_led.setData(m_buffer);
    m_led.start();

    SmartDashboard.putBoolean("Started Light", true);

    SmartDashboard.putString("Run Periodic", "not");
    // Set the default command to turn the strip off, otherwise the last colors
    // written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    setDefaultCommand(runPattern(LEDPattern.solid(elevatorColor)));

    startTime = new Date();
  }
  public void targetSearch() {
    elevatorColor = (Color.kYellow);
    elevatorBlink=false;
  }
  public void targetFound() {
    elevatorColor = (Color.kYellow);
    elevatorBlink=true;
    
  }
  public void targetLocked() {
    elevatorColor = (Color.kRed);
    elevatorBlink=false;
  }
  public void targetOff() {
    elevatorColor = (Color.kGreen);
    elevatorBlink = false;
  }

  public void elevatorUp() {
    elevatorHeight = elevatorHeight < 3 ? elevatorHeight + 1 : elevatorHeight;
  }

  public void elevatorDown() {
    elevatorHeight = elevatorHeight > 0 ? elevatorHeight - 1 : elevatorHeight;

  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to
    // display
    m_led.setData(m_buffer);
  }

  int knightRiderSpot = 0;
  int direction = 1;

  /**
   * Creates a command t hat runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> {
      for (var i = 0; i < 4; i++) {
        
        LEDPattern elevatorBase = LEDPattern.solid(elevatorColor);
        if(elevatorBlink ){
        elevatorBase = elevatorBase.blink(Seconds.of(0.3), Seconds.of(0.3));
        }
        
        if (i < elevatorHeight + 1) {
        

          elevatorBase.applyTo(elevatorBrackets[i]);
        } else {
          LEDPattern b = LEDPattern.solid(Color.kBlack);

          b.applyTo(elevatorBrackets[i]);
        }


      }
      LEDPattern spacerpattern = LEDPattern.solid(Color.kOrangeRed);
      spacerpattern.applyTo(spacersection);
      spacerpattern.applyTo(spacersection2);


      
      if (direction == 1) {
        if (knightRiderSpot < 9)
          knightRiderSpot++;
        else
          direction = -1;
      } else {
        if (knightRiderSpot > 0)
          knightRiderSpot--;
        else
          direction = 1;
      }

      // Create an LED pattern that displays a red-to-blue gradient, breathing at a 2
      // second period (0.5 Hz)
      // Distance ledSpacing = Meters.of(1 / 120.0);
      // LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous,
      // Color.kWhite, Color.kRed);
      // LEDPattern patternscroll =
      // base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
      // LEDPattern absolute =
      // base.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), ledSpacing);
      // patternscroll.applyTo(utilitysection);

      LEDPattern base = LEDPattern.solid(Color.kBlack);
      base.applyTo(utilitysection);

      //
      int scale = 15;
      int seconds = (int) (((new Date()).getTime() - startTime.getTime()) / (1000 / scale));

      if (seconds >= 10) {
        reversed = !reversed;
        startTime = (new Date());

        seconds = 0;
      }

    
      if (reversed) {
        //if (10-seconds+1<11)utilitysection.setLED((10-seconds+1), new Color(255,10,10));
        utilitysection.setLED(10 - seconds, Color.kRed);
        if (10-seconds+1<11)utilitysection.setLED((10-seconds+1), new Color(75,0,0) );
        if (10-seconds+2<11)utilitysection.setLED((10-seconds+2), new Color(25,0,0) );
        if (10-seconds+3<11)utilitysection.setLED((10-seconds+3), new Color(5,0,0) );
      } else {
        utilitysection.setLED(seconds, Color.kRed);
        if (seconds-1>-1)utilitysection.setLED((seconds-1), new Color(75,0,0) );
        if (seconds-2>-1)utilitysection.setLED((seconds-2), new Color(25,0,0) );
        if (seconds-3>-1)utilitysection.setLED((seconds-3), new Color(5,0,0) );
      }

      SmartDashboard.putString("Run Periodic", "ran");
      // pattern.applyTo(m_buffer);
      // m_led.setData(m_buffer);
      // m_led.start();
    });
  }

    
  public Command ElevatorUpCmd() {
    return runOnce(
        () -> {
          elevatorUp();
        });
  }

  public Command ElevatorDownCmd() {
    return runOnce(
        () -> {
          elevatorDown();
        });
  }
}
