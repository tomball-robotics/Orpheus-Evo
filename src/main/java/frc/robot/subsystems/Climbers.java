// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climbers extends SubsystemBase {
  /** Creates a new Climbers. */
  TalonFX climberPullR;
  TalonFX climberPullL;

  DutyCycleOut dutyCycle = new DutyCycleOut(Constants.CLIMBER.CLIMBER_SPEED);


  public Climbers() {
    if(Constants.CLIMBER.RIGHT_CLIMBER_BROKEN){
      climberPullR = new TalonFX(Constants.CLIMBER.RIGHT_MOTOR_ID);
    }
    if(Constants.CLIMBER.LEFT_CLIMBER_BROKEN){
      climberPullL = new TalonFX(Constants.CLIMBER.LEFT_MOTOR_ID);
    }
  }

  public void setClimberSpeed(double output){
    dutyCycle.Output = output;
    try{
      climberPullR.setControl(dutyCycle);
    }catch(Exception e){}
    try{
      climberPullL.setControl(dutyCycle);
    }catch(Exception e){}
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Target Speed", dutyCycle.Output);
    SmartDashboard.putBoolean("Right Climber Working", climberPullR != null);
    SmartDashboard.putBoolean("Left Climber Working", climberPullL != null);
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