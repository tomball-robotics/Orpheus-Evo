// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  TalonFX topRoller;
  TalonFX bottomRoller;

  VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(Constants.SHOOTER.TARGET_VELOCITY);
  CoastOut coastOut = new CoastOut();

  public Shooter() {
    topRoller = new TalonFX(Constants.SHOOTER.TOP_ROLLER_MOTOR_ID);
    bottomRoller = new TalonFX(Constants.SHOOTER.BOTTOM_ROLLER_MOTOR_ID);

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    Slot0Configs slot0 = cfg.Slot0; // TODO: Tune PID Constants
    slot0.kP = 0;
    slot0.kI = 0;
    slot0.kD = 0;

    StatusCode topStatus = StatusCode.StatusCodeNotInitialized;
    StatusCode bottomStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      topStatus = topRoller.getConfigurator().apply(cfg);
      bottomStatus = bottomRoller.getConfigurator().apply(cfg);
      if (topStatus.isOK() && bottomStatus.isOK()) break;
    }
    if (!(topStatus.isOK() && bottomStatus.isOK())) {
      System.out.println("Could not configure device. Top error: " + topStatus.toString() + " Bottom error: " + bottomStatus.toString());
    }

    topRoller.setControl(coastOut);
    bottomRoller.setControl(coastOut);
  }

  /**
   * Sets the target velocity for the duty cycle, then applies that to the rollers.
   * 
   * @param velocity The target velocity in revolutions per second (rps).
   */
  public void runShooter(double velocity) {
    velocityDutyCycle.Velocity = velocity; // Set target velocity
    topRoller.setControl(velocityDutyCycle); // Switch to velocity control mode
    bottomRoller.setControl(velocityDutyCycle);
  }

  /**
   * Sets the control mode for both shooter rollers to coast.
   */
  public void stopShooter() {
    topRoller.setControl(coastOut);
    bottomRoller.setControl(coastOut);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Top Roller RPS", topRoller.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Bottom Roller RPS", bottomRoller.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Duty Cycle Velocity", velocityDutyCycle.Velocity);
    SmartDashboard.putBoolean("Shooter/Rollers in Coast", topRoller.getControlMode().getName() == coastOut.getName());
    SmartDashboard.putBoolean("Shooter/Rollers in Velocity", topRoller.getControlMode().getName() == velocityDutyCycle.getName());
  }
}
