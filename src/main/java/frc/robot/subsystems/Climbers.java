// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

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
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.CLIMBER.MAX_EXTENSION_ROTATIONS;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.CLIMBER.MIN_RETRACTION_ROTATIONS;
    
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = climber.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply climber configs, error code: " + status.toString());
    }
  }

  public Command setClimberSpeed(DoubleSupplier leftOutput, DoubleSupplier rightOutput){
    return new Command () {
      {
        addRequirements(Climbers.this);
      }

      @Override
      public void initialize() {}

      @Override
      public void execute() {
        leftClimber.setControl(leftDutyCycle.withOutput(Constants.CLIMBER.LEFT_CLIMBER_DISABLED ? 0 : leftOutput.getAsDouble()));
        rightClimber.setControl(rightDutyCycle.withOutput(Constants.CLIMBER.RIGHT_CLIMBER_DISABLED ? 0 : rightOutput.getAsDouble()));
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Climbers/Left Climber Enabled", !Constants.CLIMBER.LEFT_CLIMBER_DISABLED);
    SmartDashboard.putBoolean("Climbers/Right Climber Enabled", !Constants.CLIMBER.RIGHT_CLIMBER_DISABLED);

    boolean leftClimberAtLimit = 
      Math.abs(leftClimber.getPosition().getValueAsDouble() - Constants.CLIMBER.MAX_EXTENSION_ROTATIONS) <= Constants.CLIMBER.ERROR ||
      Math.abs(leftClimber.getPosition().getValueAsDouble() - Constants.CLIMBER.MIN_RETRACTION_ROTATIONS) <= Constants.CLIMBER.ERROR;

    boolean rightClimberAtLimit = 
      Math.abs(rightClimber.getPosition().getValueAsDouble() - Constants.CLIMBER.MAX_EXTENSION_ROTATIONS) <= Constants.CLIMBER.ERROR ||
      Math.abs(rightClimber.getPosition().getValueAsDouble() - Constants.CLIMBER.MIN_RETRACTION_ROTATIONS) <= Constants.CLIMBER.ERROR;

    SmartDashboard.putBoolean("Climbers/Left Climber at Limit", leftClimberAtLimit);
    SmartDashboard.putBoolean("Climbers/Right Climber at Limit", rightClimberAtLimit);
    
    SmartDashboard.putNumber("Climbers/Left Climber Position", leftClimber.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climbers/Right Climber Position", rightClimber.getPosition().getValueAsDouble());
  }
  
}