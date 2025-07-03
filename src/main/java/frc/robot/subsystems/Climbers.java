// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbers extends SubsystemBase {
  /** Creates a new Climbers. */
  public Climbers() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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