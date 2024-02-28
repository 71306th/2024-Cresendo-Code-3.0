// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private int counter = 0;
  private double time;
  private boolean oneTime = false;
  private boolean run = false;
  
  public LED() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(Constants.LED.LEDConstants.ledPort);
    
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(Constants.LED.LEDConstants.ledLength);
    m_led.setLength(m_ledBuffer.getLength());
    
    // Set the data
    m_led.setLength(Constants.LED.LEDConstants.ledLength);
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    if(Constants.LED.LEDVariables.rainbow){
      time = Timer.getFPGATimestamp();
      if((time - (time%0.001))%0.125==0 && oneTime==false) {
        run = true;
        oneTime = true;
      }else if((time - (time%0.01))%0.25!=0){
        oneTime = false;
      }
      if(run){
        m_led.setLength(Constants.LED.LEDConstants.ledLength);
        m_ledBuffer = new AddressableLEDBuffer(Constants.LED.LEDConstants.ledLength);
        for(int i=Constants.LED.LEDConstants.ledLength-1;i>=0;i--){
          if(i%6 == (0+counter)%6){
            m_ledBuffer.setRGB(i, 255, 0, 255);
          } else if (i%6 == (1+counter)%6) {
            m_ledBuffer.setRGB(i, 255, 0, 0);
          } else if (i%6 == (2+counter)%6) {
            m_ledBuffer.setRGB(i, 255, 255, 0);
          } else if (i%6 == (3+counter)%6) {
            m_ledBuffer.setRGB(i, 0, 255, 0);
          } else if (i%6 == (4+counter)%6) {
            m_ledBuffer.setRGB(i, 0, 255, 255);
          } else if (i%6 == (5+counter)%6) {
            m_ledBuffer.setRGB(i, 0, 0, 255);
          }
        }
        m_led.setData(m_ledBuffer);
        m_led.start();
        counter++;
        run = false;
      }
    }else{
      if(Constants.Upper.UpperVariables.hasNote){
        for(int i=0;i<Constants.LED.LEDConstants.ledLength;i++) {
            m_ledBuffer.setRGB(i, 255, 95, 48);
          }
      }else{
        if(Constants.LED.LEDConstants.allianceColor == "blue") {
          for(int i=0;i<Constants.LED.LEDConstants.ledLength;i++) {
            m_ledBuffer.setRGB(i, 0, 0, 255);
          }
        }else if(Constants.LED.LEDConstants.allianceColor == "red") {
          for(int i=0;i<Constants.LED.LEDConstants.ledLength;i++) {
            m_ledBuffer.setRGB(i, 255, 0, 0);
          }
        }else{
          for(int i=0;i<Constants.LED.LEDConstants.ledLength;i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
          }
        }
      }
    }
  }
}
