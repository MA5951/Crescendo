// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.led.SolidColorPattern;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.PortMap.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class LED extends SubsystemBase {

  private static LED led;

  private AddressableLED leds;
  private AddressableLEDBuffer ledBuffer;

  public enum HPANIMATIONS {
    AMPLIFY,
    COLAB,
    GROUND,
    SOURCE,

  }
  private HPANIMATIONS lasAnimation;

  private HPANIMATIONS activeAnimation;
  private double startAmpTime = 0;
  private double startCoOpTime = 0;
  private int firstHue = 0;
  private double lastChange;
  private boolean on;

  private MAShuffleboard board;

  public LED() {
    leds = new AddressableLED(PortMap.LED.ledPort);
    ledBuffer = new AddressableLEDBuffer(LedConstants.ledLength);
    leds.setLength(ledBuffer.getLength());
    leds.setData(ledBuffer);
    leds.start();

    board = new MAShuffleboard("LED");
  }

  public void setSolidColor(Color color) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }

    leds.setData(ledBuffer);
  }

  public void setSolidColorDrivers(Color color) {
    for (var i = 0; i < LedConstants.driversLedLength; i++) {
      ledBuffer.setLED(i, color);
    }

    leds.setData(ledBuffer);
  }

  public void setSolidColorHP(Color color) {
    for (var i = LedConstants.driversLedLength + 1; i < LedConstants.hpLedLength; i++) {
      ledBuffer.setLED(i, color);
    }

    leds.setData(ledBuffer);
  }

  public void rainbowColorPatternDrivers() {
    int currentHue;
    int length = LedConstants.driversLedLength;

    for (int i = 0; i < length; i++) {
        currentHue = (firstHue + (i * 180 / length)) % 180;
        ledBuffer.setHSV(i, currentHue, 255, 128);
    }

    firstHue = (firstHue + 3) % 180;
  }

  public void rainbowColorPatternHP() {
    int currentHue;
    int length = LedConstants.hpLedLength;

    for (int i = LedConstants.driversLedLength + 1; i < length; i++) {
        currentHue = (firstHue + (i * 180 / length)) % 180;
        ledBuffer.setHSV(i, currentHue, 255, 128);
    }

    firstHue = (firstHue + 3) % 180;
  }

  public void blinkColorPatternDrivers( double interval,Color colorOne, Color colorTwo) {
    double timestamp = Timer.getFPGATimestamp();

    if (timestamp - lastChange > interval) {
        on = !on;
        lastChange = timestamp;
    }
    if (on) {
        setSolidColorDrivers(colorTwo);
    }
    else {
        setSolidColorDrivers(colorTwo);
    }
  }

  public void blinkColorPatternHP( double interval,Color colorOne, Color colorTwo) {
    double timestamp = Timer.getFPGATimestamp();

    if (timestamp - lastChange > interval) {
        on = !on;
        lastChange = timestamp;
    }
    if (on) {
        setSolidColorHP(colorTwo);
    }
    else {
        setSolidColorHP(colorTwo);
    }
  }

  public void waveColorPatternDrivers(int period , int numColors , Color[] colors) {
    double elapsedTime = Timer.getFPGATimestamp() % period;
    double progress = elapsedTime / period;
    int numLeds = LedConstants.driversLedLength;

    for (int i = 0; i < numLeds; i++) {
      double position = (double) i / (double) numLeds;
      double wavePosition = (position + progress) % 1.0;
      int colorIndex = (int) (wavePosition * numColors);
      
      Color currentColor = colors[colorIndex];
      ledBuffer.setLED(i, currentColor);
    }
  }

  public void waveColorPatternHP(int period , int numColors , Color[] colors) {
    double elapsedTime = Timer.getFPGATimestamp() % period;
    double progress = elapsedTime / period;
    int numLeds = LedConstants.hpLedLength;

    for (int i = LedConstants.driversLedLength + 1; i < numLeds; i++) {
      double position = (double) i / (double) numLeds;
      double wavePosition = (position + progress) % 1.0;
      int colorIndex = (int) (wavePosition * numColors);
      
      Color currentColor = colors[colorIndex];
      ledBuffer.setLED(i, currentColor);
    }
  }

  public void smoothWaveColorPatternDrivers(int numColors, double period, double speed, Color[] colors) {
    double elapsedTime = Timer.getFPGATimestamp();

    for (int i = 0; i < LedConstants.driversLedLength; i++) {
      double position = ((double) i / LedConstants.driversLedLength) + (elapsedTime * speed / period);
      double progress = position - (int) position;

      int startColorIndex = (int) (position % numColors);
      int endColorIndex = (startColorIndex + 1) % numColors;
      Color startColor = colors[startColorIndex];
      Color endColor = colors[endColorIndex];

      Color currentColor = new Color(
              startColor.red + (endColor.red - startColor.red) * progress,
              startColor.green + (endColor.green - startColor.green) * progress,
              startColor.blue + (endColor.blue - startColor.blue) * progress
      );

      ledBuffer.setLED(i, currentColor);
    }
  }

  public void smoothWaveColorPatternHP(int numColors, double period, double speed, Color[] colors) {
    double elapsedTime = Timer.getFPGATimestamp();

    for (int i = LedConstants.driversLedLength + 1; i < LedConstants.hpLedLength; i++) {
      double position = ((double) i / LedConstants.hpLedLength) + (elapsedTime * speed / period);
      double progress = position - (int) position;

      int startColorIndex = (int) (position % numColors);
      int endColorIndex = (startColorIndex + 1) % numColors;
      Color startColor = colors[startColorIndex];
      Color endColor = colors[endColorIndex];

      Color currentColor = new Color(
              startColor.red + (endColor.red - startColor.red) * progress,
              startColor.green + (endColor.green - startColor.green) * progress,
              startColor.blue + (endColor.blue - startColor.blue) * progress
      );

      ledBuffer.setLED(i, currentColor);
    }
  }

  public void smoothWaveColorPattern(int numColors, double period, double speed, Color[] colors) {
    double elapsedTime = Timer.getFPGATimestamp();

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      double position = ((double) i / ledBuffer.getLength()) + (elapsedTime * speed / period);
      double progress = position - (int) position;

      int startColorIndex = (int) (position % numColors);
      int endColorIndex = (startColorIndex + 1) % numColors;
      Color startColor = colors[startColorIndex];
      Color endColor = colors[endColorIndex];

      Color currentColor = new Color(
              startColor.red + (endColor.red - startColor.red) * progress,
              startColor.green + (endColor.green - startColor.green) * progress,
              startColor.blue + (endColor.blue - startColor.blue) * progress
      );

      ledBuffer.setLED(i, currentColor);
    }
  }

  public void blinkColorPattern( double interval,Color colorOne, Color colorTwo) {
    double timestamp = Timer.getFPGATimestamp();

    if (timestamp - lastChange > interval) {
        on = !on;
        lastChange = timestamp;
    }
    if (on) {
        setSolidColor(colorTwo);
    }
    else {
        setSolidColor(colorTwo);
    }
  }

  public void updateLeds() {
    leds.setData(ledBuffer);
  }

  public void setAllianceColor() {
    if ( DriverStation.isFMSAttached()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
      smoothWaveColorPattern(2, 1, 0.2, new Color[] {LedConstants.BLACK , LedConstants.BLUE});
    } else if ( DriverStation.getAlliance().get() == Alliance.Red) {
      smoothWaveColorPattern(2, 1, 0.2, new Color[] {LedConstants.BLACK , LedConstants.RED});
    } else {
      smoothWaveColorPattern(2, 1, 0.2, new Color[] {LedConstants.BLACK , LedConstants.PURPLE});
    }
    } else {
      blinkColorPattern(1, LedConstants.PURPLE, LedConstants.BLACK);
    }
  }

  public static LED getInstance() {
    if (led == null) {
      led = new LED();
    }
    return led;
  }

  public void activateAmp() {
    lasAnimation = activeAnimation;
    activeAnimation = HPANIMATIONS.AMPLIFY;
    startAmpTime = Timer.getFPGATimestamp();
  }

  public void activateCoalition() {
    lasAnimation = activeAnimation;
    activeAnimation = HPANIMATIONS.COLAB;
    startCoOpTime = Timer.getFPGATimestamp();
  }

  public void runDriversAnimations() {
    if (SwerveDrivetrainSubsystem.getInstance().canShoot()) {
      setSolidColorDrivers(LedConstants.Ring);
    } else if (Intake.getInstance().isGamePieceInIntake()) {
      setSolidColorDrivers(LedConstants.GREEN);
    } else {
      setSolidColor(LedConstants.BLUE);
    }
  }

  public void runHpAnimations() {
    if (activeAnimation == HPANIMATIONS.AMPLIFY) {
      smoothWaveColorPatternHP(2, 1, 1, new Color[] {LedConstants.BLUE , LedConstants.BLACK});
    } else if (activeAnimation == HPANIMATIONS.COLAB) {
      smoothWaveColorPatternHP(2, 1, 1, new Color[] {LedConstants.Ring , LedConstants.BLACK});
    } else if (activeAnimation == HPANIMATIONS.GROUND) {
      setSolidColorHP(LedConstants.BLUE);
    } else if (activeAnimation == HPANIMATIONS.SOURCE) {
      setSolidColorHP(LedConstants.MAcolor);
    }
  }

  public void setAnimation(HPANIMATIONS animation) {
    activeAnimation = animation;
  }

  @Override
  public void periodic() {
    // if (!DriverStation.isEnabled()) {
    //   setAllianceColor();
    // } else if (DriverStation.isAutonomous()) {
    //   //Auto animation
    // } else {
    //   runDriversAnimations();
    // }

    setSolidColor(LedConstants.GREEN);

    updateLeds();
  }
}
