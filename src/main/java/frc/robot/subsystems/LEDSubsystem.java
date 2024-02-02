// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.ListIterator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.LedConstants;
import frc.utils.led.TrobotAddressableLED;
import frc.utils.led.TrobotAddressableLEDPattern;
import frc.utils.led.patterns.matrix.ColorSoundMeter;
import frc.utils.led.patterns.matrix.MultiColorMeter;
import frc.utils.led.patterns.strip.RainbowPattern;
import frc.utils.led.patterns.strip.SolidColorPattern;

public class LEDSubsystem extends SubsystemBase {
  private TrobotAddressableLED m_led = new TrobotAddressableLED(LedConstants.kLedPWMPort,
                                       LedConstants.kLedCount);

  private TrobotAddressableLEDPattern m_bluePattern = new SolidColorPattern(Color.kBlue);
	private TrobotAddressableLEDPattern m_redPattern = new SolidColorPattern(Color.kRed);
	private TrobotAddressableLEDPattern m_purplePattern = new SolidColorPattern(Color.kPurple);
	private TrobotAddressableLEDPattern m_yellowPattern = new SolidColorPattern(Color.kYellow);
	private TrobotAddressableLEDPattern m_disabledPattern = new RainbowPattern();
	private TrobotAddressableLEDPattern m_rainbowMeter; 
	private TrobotAddressableLEDPattern m_blueSoundMeter; 
	private TrobotAddressableLEDPattern m_redSoundMeter; 
  
  private TrobotAddressableLEDPattern m_currentPattern;
  private List<TrobotAddressableLEDPattern> m_patternList;
  private ListIterator<TrobotAddressableLEDPattern> m_patternIterator;
  private CommandXboxController m_controller;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(CommandXboxController controller) {
    m_controller = controller;
    m_rainbowMeter = new MultiColorMeter(()->m_controller.getRightTriggerAxis());
    m_blueSoundMeter = new ColorSoundMeter(()->m_controller.getRightTriggerAxis(),Color.kBlue);
    m_redSoundMeter = new ColorSoundMeter(()->m_controller.getRightTriggerAxis(),Color.kRed);

    m_patternList = new ArrayList<TrobotAddressableLEDPattern>
                        (Arrays.asList(m_redPattern,m_bluePattern,
                                       m_yellowPattern, m_purplePattern,
                                       m_disabledPattern,m_rainbowMeter,m_blueSoundMeter,m_redSoundMeter));


    m_patternIterator = m_patternList.listIterator();
    
    m_currentPattern =  m_redPattern;
  }


  public void nextPattern() {
       if (!m_patternIterator.hasNext()) {
        m_patternIterator = m_patternList.listIterator();
      }
      m_currentPattern = m_patternIterator.next();
  }

//   public void setConePattern() {
//     setPattern(m_yellowPattern);
//   }

//   public void setCubePattern() {
//     setPattern(m_purplePattern);
//   }

//   public void raiseTheNoise(double level) {
//     setPattern(new SolidColorPattern(Color.kBlue));
//   }

//   public void setPattern(TrobotAddressableLEDPattern pattern) {
//     m_currentPattern = pattern;
//     var patternExists = m_patternList.contains(pattern);
//     if (patternExists) {
//         m_patternIterator = m_patternList.listIterator(
//                                       m_patternList.indexOf(pattern));
//     }
//   }

  @Override
  public void periodic() {
    m_led.setPattern(m_currentPattern);
    // This method will be called once per scheduler run
  }
}
