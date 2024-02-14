// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.led.AddressableLEDController;
import com.ma5951.utils.led.BlinkingColorPattern;
import com.ma5951.utils.led.BreathingColorPattern;
import com.ma5951.utils.led.BreathingTripleColorPattern;
import com.ma5951.utils.led.EvenOddColorPattern;
import com.ma5951.utils.led.RainbowColorPatterSimultaneously;
import com.ma5951.utils.led.RainbowColorPattern;
import com.ma5951.utils.led.SolidColorPattern;
import com.ma5951.utils.led.WaveBlinkColorPattern;
import com.ma5951.utils.led.SmoothColorTransitionPattern;
import com.ma5951.utils.led.SmoothWaveColorPattern;
import com.ma5951.utils.led.WavePattern;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private static LED led;

  private AddressableLED addressableLED;
  private AddressableLEDBuffer addressableLEDBuffer;

  AddressableLEDController ledController;
  SolidColorPattern solidColorPattern;
  RainbowColorPattern rainbowColorPattern;
  RainbowColorPatterSimultaneously rainbowColorPatterSimultaneously;
  BlinkingColorPattern blinkingColorPattern;
  BreathingColorPattern breathingColorPattern;
  BreathingTripleColorPattern breathingTripleColorPattern;
  SmoothColorTransitionPattern smoothColorTransitionPattern;
  WavePattern wavePattern;
  SmoothWaveColorPattern smoothWaveColorPattern;
  WaveBlinkColorPattern waveBlinkColorPattern;
  EvenOddColorPattern evenOddColorPattern;
  
  private boolean activateAmp = false;
  private boolean activateCoOp = false;

  private MAShuffleboard board;

  public LED() {

    addressableLED = new AddressableLED(PortMap.LED.ledPort);
    addressableLEDBuffer = new AddressableLEDBuffer(LedConstants.ledLength);
    addressableLED.setLength(addressableLEDBuffer.getLength());
    addressableLED.setData(addressableLEDBuffer);
    addressableLED.start();
    //ledController = new AddressableLEDController(PortMap.LED.ledPort, LedConstants.ledLength);

    board = new MAShuffleboard("LED");
  }

  public void setSolidColor(Color color) {
    solidColorPattern.setColor(color);
    ledController.setAddressableLEDPattern(solidColorPattern);
  }

  public void setColor(Color color) {
    for (var i = 0; i < addressableLEDBuffer.getLength(); i++) {
      addressableLEDBuffer.setLED(i, color);
    }

    addressableLED.setData(addressableLEDBuffer);
  }

  public void setSmoothWave(int numColors, double period, double speed, Color[] colors) {
    smoothWaveColorPattern.setParameters(numColors, period, speed, colors);
    ledController.setAddressableLEDPattern(smoothWaveColorPattern);
  }

  public void setRainbow() {
    ledController.setAddressableLEDPattern(rainbowColorPattern);
  }

  public void setSmoothColorTransition(Color color, Color color2, double interval) {
    smoothColorTransitionPattern.setParameters(color, color2, interval);
    ledController.setAddressableLEDPattern(smoothColorTransitionPattern);
  }
  
  public void setFullRainbow() {
    ledController.setAddressableLEDPattern(rainbowColorPatterSimultaneously);
  }


  public void setWave(int numColors, double period, double speed, Color[] colors) {
    wavePattern.setParameters(numColors, period, speed, colors);
    ledController.setAddressableLEDPattern(wavePattern);
  }
  
  public void setEvenOdd(Color color, Color color2, double lenght) {
    evenOddColorPattern.setParameters(color, color2, lenght);
    ledController.setAddressableLEDPattern(evenOddColorPattern);
  }


  public void setWaveBlink(Color color, Color color2, double interval) {
    waveBlinkColorPattern.setParameters(color, color2, interval);
    ledController.setAddressableLEDPattern(waveBlinkColorPattern);
  }


  public void setBlinking(Color color, Color color2, double interval) {
    blinkingColorPattern.setParameters(color, color2, interval);
    ledController.setAddressableLEDPattern(blinkingColorPattern);
  }

  public void setBreathing(Color color, double interval){
    breathingColorPattern.setParameters(color, interval);
    ledController.setAddressableLEDPattern(breathingColorPattern);
  }

  public void setBreathingTriple(Color color, Color color2, double interval){
    breathingTripleColorPattern.setParameters(color, color2, interval);
    ledController.setAddressableLEDPattern(breathingTripleColorPattern);
  }

  public void setAllianceColor() {
    if (DriverStation.isFMSAttached()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        setSmoothWave(2, 1, 1, new Color [] {LedConstants.RED, LedConstants.BLACK});
      } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
        setSmoothWave(2, 1, 1, new Color [] {LedConstants.BLUE, LedConstants.BLACK});
      } else {
        setSmoothWave(2, 1, 1, new Color [] {LedConstants.PURPLE, LedConstants.BLACK});
      }
    } else {
      setBlinking(LedConstants.BLACK, LedConstants.RED, 2);
    }
  }

  public static LED getInstance() {
    if (led == null) {
      led = new LED();
    }
    return led;
  }

  public void activateAmp() {
    activateAmp = true;
  }

  public void activateCoalition() {
    activateCoOp = true;
  }

  @Override
  public void periodic() {
    // if (DriverStation.isDisabled()) {
    //   setAllianceColor();
    // } 
    // else {
    //   if (activateAmp) {
    //     if (Timer.getFPGATimestamp() - startAmpTime < LedConstants.activateTime) {
    //       setBlinking(LedConstants.BLUE, LedConstants.WHITE, 0.3);
    //     } else if (Timer.getFPGATimestamp() - startAmpTime < 10 && Timer.getFPGATimestamp() - startAmpTime > LedConstants.activateTime) {
    //       if (Intake.getInstance().isGamePieceInIntake()) {
    //         setSmoothWave(2, 1, 1, new Color [] {LedConstants.GREEN, LedConstants.BLUE}); 
    //       } else if (!Intake.getInstance().isGamePieceInIntake()) {
    //         setSmoothWave(2, 1, 1, new Color [] {LedConstants.Ring, LedConstants.BLUE});
    //       }
    //     } else {
    //       activateAmp = false;
    //     }
    //   } else if (activateCoOp) {
    //     if (Timer.getFPGATimestamp() - startCoOpTime < LedConstants.activateTime) {
    //       setBlinking(LedConstants.YELLOW, LedConstants.WHITE, 0.3);
    //     } else if (Timer.getFPGATimestamp() - startCoOpTime < 10 && Timer.getFPGATimestamp() - startCoOpTime > LedConstants.activateTime) {
    //       if (Intake.getInstance().isGamePieceInIntake()) {
    //         setSmoothWave(2, 1, 1, new Color [] {LedConstants.GREEN, LedConstants.YELLOW}); 
    //       } else if (!Intake.getInstance().isGamePieceInIntake()) {
    //         setSmoothWave(2, 1, 1, new Color [] {LedConstants.Ring, LedConstants.YELLOW});
    //       }
    //     } else {
    //       activateCoOp = false;
    //     }
    //   }  
    //   else {
    //     if (Intake.getInstance().isGamePieceInIntake()) {
    //       setSmoothWave(2, 1, 1, new Color [] {LedConstants.GREEN, LedConstants.WHITE}); 
    //     } else if (!Intake.getInstance().isGamePieceInIntake()) {
    //       setSmoothWave(2, 1, 1, new Color [] {LedConstants.Ring, LedConstants.WHITE});
    //     }
    //   }
    // }
    if (SwerveDrivetrainSubsystem.getInstance().canShoot()) {
      setColor(LedConstants.Ring);
    } else if (Intake.getInstance().isGamePieceInIntake()) {
      setColor(LedConstants.GREEN);
    } else if (RobotContainer.IsFloor()){
      setColor(LedConstants.BLUE);
    } else {
      setColor(LedConstants.MAcolor);
    }
    board.addBoolean("activateAmp", activateAmp);
    board.addBoolean("activateCoOp", activateCoOp);
  }
}
