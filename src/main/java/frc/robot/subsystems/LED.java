// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {

  static Spark led;

  public LED() {
    led = new Spark(Constants.LED.ledID);
  }

  public static void setColor(double color) {
    led.set(color);
  }

  public static void setDisabled() {
    led.set(.61);
  }

  public static void setEnabled() {
    led.set(.61);
  }

  public static void noteIndexed() {
    led.set(.77);
  }

  public static void intakeAtVelocity() {
    led.set(-0.05);
  }

  public static void intakeRunning(){
    led.set(-0.25);
  }

  public static void shooterRunning(){
    led.set(-.15);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LED Output Value", led.get());
  }
}
