package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {

  private static Spark led;
  private static double currentColor = 0.0;
  private static String currentState = "Unknown";

  public LED() {
    led = new Spark(Constants.LED.ledID);
    setDisabled();  // Initial state on boot
  }

  private static void applyColor(double color, String stateName) {
    if (currentColor != color) {
      currentColor = color;
      currentState = stateName;
      led.set(color);
    }
  }

  public static void setDisabled() {
    applyColor(0.61, "Disabled");
  }

  public static void setEnabled() {
    applyColor(0.61, "Enabled");
  }

  public static void noteIndexed() {
    applyColor(0.77, "Note Indexed");
  }

  public static void intakeAtVelocity() {
    applyColor(-0.05, "Intake At Velocity");
  }

  public static void intakeRunning() {
    applyColor(-0.25, "Intake Running");
  }

  public static void shooterRunning() {
    applyColor(-0.15, "Shooter Running");
  }

  public static boolean isInEnabledState() {
    return currentState.equals("Enabled");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LED/Output Value", currentColor);
    SmartDashboard.putString("LED/Current State", currentState);
  }
}
