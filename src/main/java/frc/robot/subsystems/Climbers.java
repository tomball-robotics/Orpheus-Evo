// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climbers extends SubsystemBase {

  TalonFX leftClimber;
  TalonFX rightClimber;

  DutyCycleOut leftDutyCycle = new DutyCycleOut(0);
  DutyCycleOut rightDutyCycle = new DutyCycleOut(0);

  public Climbers() {
    leftClimber = new TalonFX(Constants.CLIMBER.RIGHT_MOTOR_ID);
    rightClimber = new TalonFX(Constants.CLIMBER.LEFT_MOTOR_ID);
  }

  public Command setLeftClimberSpeed(double output){
    return new Command () {
      @Override
      public void initialize() {
        leftDutyCycle.Output = Constants.CLIMBER.LEFT_CLIMBER_BROKEN ? 0 : output;
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  public Command setRightClimberSpeed(double output){
    return new Command () {
      @Override
      public void initialize() {
        rightDutyCycle.Output = Constants.CLIMBER.RIGHT_CLIMBER_BROKEN ? 0 : output;
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Left Climber Broken", Constants.CLIMBER.LEFT_CLIMBER_BROKEN);
    SmartDashboard.putBoolean("Right Climber Broken", Constants.CLIMBER.RIGHT_CLIMBER_BROKEN);
    SmartDashboard.putNumber("Left Climber Speed", leftDutyCycle.Output);
    SmartDashboard.putNumber("Right Climber Speed", rightDutyCycle.Output);
  }
  
}

/* FOR DANIEL
 * 
 * This subsystem will house the climber motors and their controls.
 * It will also handle the logic for climbing, such as checking if the climbers are broken,
 * and controlling the speed of the climbers.
 * 
 * You will probably need to use a dutycycleout to control the speed of the climbers,
 * so that they can be directly linked to an axis on the controller.
 * dutycycleout is where you feed it a value between -1 and 1,
 * and it will output that value to the motor, 
 * -1 being full reverse, 0 being stopped, and 1 being full forward.
 */