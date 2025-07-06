// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LEDPattern;

public class LED extends SubsystemBase {
  Spark led;
  /** Creates a new LEDs. */
  public LED() {
    led = new Spark(Constants.LED.ledID);
  }

  public void setColor(double color) {
    led.set(color);
  }

  public void setDisabled() {
    led.set(.61);
  }

  public void setEnabled() {
    led.set(.61);
  }

  public void isIntake() {
    led.set(.77);
  }

  public void intakeIsRunning(){
    led.set(-.05);
  }

  public void shooterRunning(){
    led.set(-.15);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LED Value", LEDPattern.fromValue(led.get()).name());
  }
}
