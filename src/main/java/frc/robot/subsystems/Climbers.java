// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class Climbers extends SubsystemBase {

  TalonFX leftClimber;
  TalonFX rightClimber;

  DutyCycleOut leftDutyCycle = new DutyCycleOut(0);
  DutyCycleOut rightDutyCycle = new DutyCycleOut(0);

  private final double MAX_EXTENSION_ROTATIONS = 50.0; // TODO: Tune
  private final double MIN_RETRACTION_ROTATIONS = 0.0;
  
  public Climbers() {
    leftClimber = new TalonFX(Constants.CLIMBER.RIGHT_MOTOR_ID);
    rightClimber = new TalonFX(Constants.CLIMBER.LEFT_MOTOR_ID);
    
    configureClimber(leftClimber);
    configureClimber(rightClimber);
    
    leftClimber.setPosition(0);
    rightClimber.setPosition(0);
  }

  private void configureClimber(TalonFX climber) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: Tune
    
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_EXTENSION_ROTATIONS;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_RETRACTION_ROTATIONS;
    
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = climber.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply climber configs, error code: " + status.toString());
    }
  }

  public Command setClimberSpeed(double leftOutput, double rightOutput){
    return new Command () {
      @Override
      public void initialize() {
    }

      @Override
      public void execute() {
        leftClimber.setControl(leftDutyCycle.withOutput(Constants.CLIMBER.LEFT_CLIMBER_BROKEN ? 0 : leftOutput));
        rightClimber.setControl(rightDutyCycle.withOutput(Constants.CLIMBER.RIGHT_CLIMBER_BROKEN ? 0 : rightOutput));
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  // Utility methods for position checking
  public double getLeftClimberPosition() {
    return leftClimber.getPosition().getValueAsDouble();
  }

  public double getRightClimberPosition() {
    return rightClimber.getPosition().getValueAsDouble();
  }

  public boolean isLeftClimberAtLimit() {
    double position = getLeftClimberPosition();
    return position <= MIN_RETRACTION_ROTATIONS || position >= MAX_EXTENSION_ROTATIONS;
  }

  public boolean isRightClimberAtLimit() {
    double position = getRightClimberPosition();
    return position <= MIN_RETRACTION_ROTATIONS || position >= MAX_EXTENSION_ROTATIONS;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Left Climber Broken", Constants.CLIMBER.LEFT_CLIMBER_BROKEN);
    SmartDashboard.putBoolean("Right Climber Broken", Constants.CLIMBER.RIGHT_CLIMBER_BROKEN);
    SmartDashboard.putNumber("Left Climber Speed", leftDutyCycle.Output);
    SmartDashboard.putNumber("Right Climber Speed", rightDutyCycle.Output);
    
    SmartDashboard.putNumber("Left Climber Position", getLeftClimberPosition());
    SmartDashboard.putNumber("Right Climber Position", getRightClimberPosition());
    SmartDashboard.putBoolean("Left Climber At Limit", isLeftClimberAtLimit());
    SmartDashboard.putBoolean("Right Climber At Limit", isRightClimberAtLimit());
  }
  
}