package frc.robot;

public class Constants {

    public static final class CONTROL {
        public static final int DRIVER_CONTROLLER_PORT_ID = 0;
        public static final int OPERATOR_CONTROLLER_PORT_ID = 1;
        public static final double STICK_DEADBAND = .1;
    }

    public static final class SHOOTER {
        public static final int TOP_ROLLER_MOTOR_ID = -1; // TODO: Tune
        public static final int BOTTOM_ROLLER_MOTOR_ID = -1; // TODO: Tune
        public static final double VELOCITY_P = 0; // TODO: Tune
        public static final double VELOCITY_I = 0; // TODO: Tune
        public static final double VELOCITY_D = 0; // TODO: Tune
        public static final double TARGET_VELOCITY = 0.5; // TODO: Tune
    }

    public static final class INTAKE {
        public static final int LEFT_PIVOT_MOTOR_ID = -1; // TODO: Tune
        public static final int RIGHT_PIVOT_MOTOR_ID = -1; // TODO: Tune
        public static final double PIVOT_INTAKE_ANGLE = 0.0; // TODO: Tune
        public static final double PIVOT_AMP_ANGLE = 0.0; // TODO: Tune
        public static final double PIVOT_STOW_ANGLE = 0.0; // TODO: Tune
        public static final double PIVOT_P = 0; // TODO: Tune
        public static final double PIVOT_I = 0; // TODO: Tune
        public static final double PIVOT_D = 0; // TODO: Tune

        public static final int ROLLER_MOTOR_ID = -1; // TODO: Tune
        public static final double ROLLER_VELOCITY_P = 0; // TODO: Tune
        public static final double ROLLER_VELOCITY_I = 0; // TODO: Tune
        public static final double ROLLER_VELOCITY_D = 0; // TODO: Tune
        public static final double INTAKE_ROLLER_VELOCITY = 0.0; // TODO: Tune
        public static final double AMP_ROLLER_VELOCITY = 0.0; // TODO: Tune
        public static final double FEED_ROLLER_VELOCITY = 0.0; // TODO: Tune

        public static final int ROLLER_ENCODER_ID = -1; //TODO: Tune
    }

    public static final class CLIMBER {
        public static final int LEFT_MOTOR_ID = -1; // TODO: Tune
        public static final int RIGHT_MOTOR_ID = -1; // TODO: Tune
        public static final double CLIMBER_SPEED = 0.5; // TODO: Tune
        public static final boolean LEFT_CLIMBER_BROKEN = true;
        public static final boolean RIGHT_CLIMBER_BROKEN = false;
    }

    public static final class LED {
        public static final int ledID = -1; //TODO: Tune
    }

}
