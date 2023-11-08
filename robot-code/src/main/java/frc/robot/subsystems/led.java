// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.*;
import edu.wpi.first.wpilibj.AddressableLED;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
public class led extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private Timer timer;
  static led LEDInstance = null;
  int RedRed = 100;
  int RedGreen = 0;
  int RedBlue = 0;
  int redVar;
  int blueVar;
  int greenVar;
  Random r = new Random();

  boolean spin = false;

  int OrangeRed = 255;
  int OrangeGreen = 127;
  int OrangeBlue = 0;


  int YellowRed = 255;
  int YellowGreen = 255;
  int YellowBlue = 0;


  int GreenRed = 0;
  int GreenGreen = 255;
  int GreenBlue = 0;


  int BlueRed = 0;
  int BlueGreen = 0;
  int BlueBlue = 255;


  int PurpleRed = 75;
  int PurpleGreen = 0;
  int PurpleBlue = 130;

  /** Creates a new led. */
  public led() {
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(300); //300 should be the length
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start(); //starts the leds
     timer = new Timer();
  }

  public int getLength(){
    return m_ledBuffer.getLength();
  }
  //-------------------------------------THEMES----------------------------------------------------------//
  public void resetled(){ //turns all the leds off
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
   }
    m_led.setData(m_ledBuffer);
      }

  public void setLed(int r, int g, int b){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_ledBuffer);
  }

  public void setIndividalLed(int i, int r, int g, int b){
    m_ledBuffer.setRGB(i, r, g, b);
    m_led.setData(m_ledBuffer);
  }

  public void CycleThroughTheRainbow(){
    
    for (int i = 0; i < m_ledBuffer.getLength(); i++){
      if (i % 18 >= 0 && i % 18 < 3){
          m_ledBuffer.setRGB(i, RedRed, RedGreen, RedBlue);
      }
      else if (i % 18 >= 3 && i % 18 < 6){
        m_ledBuffer.setRGB(i, OrangeRed, OrangeGreen, OrangeBlue);
      }
      else if (i % 18 >= 6 && i % 18 < 9){
        m_ledBuffer.setRGB(i, YellowRed, YellowGreen, YellowBlue);
      }
      else if (i % 18 >= 9 && i % 18 < 12){
        m_ledBuffer.setRGB(i, GreenRed, GreenGreen, GreenBlue);
      }
      else if (i % 18 >= 12 && i % 18 < 15){
        m_ledBuffer.setRGB(i, BlueRed, BlueGreen, BlueBlue);
      }
      else if (i % 18 >= 15 && i % 18 < 18){
        m_ledBuffer.setRGB(i, PurpleRed, PurpleGreen, PurpleBlue);
      }
      m_led.setData(m_ledBuffer);
    }
    // for (var i = 0; i < m_ledBuffer.getLength()/6; i++) {
    //   m_ledBuffer.setRGB(i, RedRed, RedGreen, RedBlue);
    // }
    // for (var i = m_ledBuffer.getLength()/6;  i < m_ledBuffer.getLength()/3; i++) {
    //   m_ledBuffer.setRGB(i, OrangeRed, OrangeGreen, OrangeBlue);
    // }
    // for (var i = m_ledBuffer.getLength()/3; i < m_ledBuffer.getLength()/2; i++) {
    //   m_ledBuffer.setRGB(i, YellowRed, YellowGreen, YellowBlue);
    // }
    // for (var i = m_ledBuffer.getLength()/2; i < 2*m_ledBuffer.getLength()/3; i++) {
    //   m_ledBuffer.setRGB(i, GreenRed, GreenGreen, GreenBlue);
    // }
    // for (var i = 2*m_ledBuffer.getLength()/3; i < 5*m_ledBuffer.getLength()/6; i++) {
    //   m_ledBuffer.setRGB(i, BlueRed, BlueGreen, BlueBlue);
    // }
    // for (var i = 5*m_ledBuffer.getLength()/6; i < m_ledBuffer.getLength(); i++) {
    //   m_ledBuffer.setRGB(i, PurpleRed, PurpleGreen, PurpleBlue);
    // }
    
  }

  public boolean SpinToWin(){
    spin = false;
    redVar = r.nextInt(255);
    greenVar = r.nextInt(255);
    blueVar = r.nextInt(255);

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, redVar, greenVar, blueVar);
   }

   if(redVar>100 && blueVar > 100){
      spin = true;
   }
    m_led.setData(m_ledBuffer);
    return spin;
  }
  public void yellow(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 255, 0);
   }
    m_led.setData(m_ledBuffer);
  }

  public void purple(){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 0, 255);
   }
    m_led.setData(m_ledBuffer);
  }
  
  //chantilly theme
  public void ChantillyTheme(){ //Chantilly color pattern; purple and white
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      if(i % 2 == 0){
        m_ledBuffer.setHSV(i, 136, 0, 60);
      } else {
        m_ledBuffer.setRGB(i, 75, 0, 130);
      }
    }
    m_led.setData(m_ledBuffer);
  }

  public void SixTwelveTheme(){ //612 color pattern; blue, yellow, and white
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      if(i % 3 == 0){
        m_ledBuffer.setHSV(i, 136, 0, 60);
      } else if(i % 3 == 1){
        m_ledBuffer.setRGB(i, 255, 0, 255);
      }else{
        m_ledBuffer.setRGB(i, 0, 0, 255);
      }
    }
    m_led.setData(m_ledBuffer);
  }
  
  //yellow sparkle
  public void SparklePhase1(){
    timer.start();
    if (timer.get() <= 1){
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      if(i % 2 == 0){
        m_ledBuffer.setRGB(i,255,255,0);
        
       }
       else {
        m_ledBuffer.setRGB(i,0,0,0);
       }
    }
  }
    
     if (timer.get() >= 1){
      for (var i = 0; i < m_ledBuffer.getLength(); i++){
        if (i % 2 == 1) {
          m_ledBuffer.setRGB(i,255,255,0);
        }
        else {
        m_ledBuffer.setRGB(i,0,0,0);
        }
      }
    }
    if (timer.get() >= 2){
      timer.reset();
    }

    m_led.setData(m_ledBuffer);
  }
  public static led getLEDInstance() {
    if (LEDInstance == null) {
      LEDInstance = new led();
    }
    return LEDInstance;
  }
  //ends here

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}