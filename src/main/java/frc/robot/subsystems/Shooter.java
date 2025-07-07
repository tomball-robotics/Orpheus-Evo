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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  TalonFX topRoller;
  TalonFX bottomRoller;

  VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);
  double velocitySetpoint = 0;
  CoastOut coastOut = new CoastOut();

  public Shooter() {
    topRoller = new TalonFX(Constants.SHOOTER.TOP_ROLLER_MOTOR_ID);
    bottomRoller = new TalonFX(Constants.SHOOTER.BOTTOM_ROLLER_MOTOR_ID);

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    Slot0Configs slot0 = cfg.Slot0; // TODO: Tune PID Constants
    slot0.kP = Constants.SHOOTER.VELOCITY_P;
    slot0.kI = Constants.SHOOTER.VELOCITY_I; 
    slot0.kD = Constants.SHOOTER.VELOCITY_D;

    StatusCode topStatus = StatusCode.StatusCodeNotInitialized;
    StatusCode bottomStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      topStatus = topRoller.getConfigurator().apply(cfg);
      bottomStatus = bottomRoller.getConfigurator().apply(cfg);
      if (topStatus.isOK() && bottomStatus.isOK()) break;
    }
    if (!(topStatus.isOK() && bottomStatus.isOK())) {
      System.out.println("Could not configure shooter motors. Top status: " + topStatus.toString() + " Bottom status: " + bottomStatus.toString());
    }

    topRoller.setControl(coastOut);
    bottomRoller.setControl(coastOut);
  }

  public Command runShooterCommand(double velocity) {
    return new Command() {
      @Override
      public void initialize() {
        topRoller.setControl(velocityDutyCycle.withVelocity(velocity));
        bottomRoller.setControl(velocityDutyCycle.withVelocity(velocity));
        velocitySetpoint = velocity; // Store the setpoint for potential future use
        System.out.println("Shooter rollers set to velocity: " + velocity);
      }

      @Override
      public boolean isFinished() {
        return false; // This command runs indefinitely until interrupted
      }

      @Override
      public void end(boolean interrupted) {
        stopShooter(); // Stop the shooter when the command ends or is interrupted
        System.out.println("Shooter rollers stopped.");
      }
    };
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
    SmartDashboard.putNumber("Shooter/Rollers Setpoint", velocitySetpoint);
    SmartDashboard.putString("Shooter/Top Roller Control Mode", topRoller.getControlMode().getName());
    SmartDashboard.putString("Shooter/Bottom Roller Control Mode", bottomRoller.getControlMode().getName());
  }
}
