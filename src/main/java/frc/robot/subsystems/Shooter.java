// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  TalonFX topRoller;
  TalonFX bottomRoller;

  VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(Constants.SHOOTER.TARGET_VELOCITY);
  CoastOut coastOut = new CoastOut();

  public Shooter() { // TODO add pid constants to the velocityDutyCycle !!!
    topRoller = new TalonFX(Constants.SHOOTER.TOP_ROLLER_MOTOR_ID);
    bottomRoller = new TalonFX(Constants.SHOOTER.BOTTOM_ROLLER_MOTOR_ID);

    topRoller.setControl(coastOut);
    bottomRoller.setControl(coastOut);
  }

  /**
   * Sets the target velocity for both shooter rollers.
   * 
   * @param velocity The target velocity in revolutions per second (rps).
   */
  public void setTargetVelocity(double velocity) {
    velocityDutyCycle.Velocity = velocity;
  }

  /**
   * Sets the control mode for both shooter rollers to velocity control.
   */
  public void setVelocityControl() {
    topRoller.setControl(velocityDutyCycle);
    bottomRoller.setControl(velocityDutyCycle);
  }

  /**
   * Sets the control mode for both shooter rollers to coast.
   */
  public void setCoastControl() {
    topRoller.setControl(coastOut);
    bottomRoller.setControl(coastOut);
  }

  /**
   * Creates a command to set the target velocity for both shooter rollers.
   * 
   *  @param velocity The target velocity in revolutions per second (rps).
   */
  public Command setVelocityCommand(double velocity) {
    return runOnce(() -> {
      setTargetVelocity(velocity);
    });
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/Top Roller RPS", topRoller.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Bottom Roller RPS", bottomRoller.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter/Target Velocity", velocityDutyCycle.Velocity);
  }
}
