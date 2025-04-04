package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    public static final int ELEVATOR_L_CAN_ID = 11; // TODO: Replace with your actual CAN ID
    public static final int ELEVATOR_R_CAN_ID = 12; // TODO: Replace with your actual CAN ID
    public static final int LOWER_LIMIT_SWITCH_PORT = 4; // TODO: Replace with your actual DIO port
    public static final int ELEVATOR_CURRENT_LIMITS = 40; // TODO: Verify or adjust this value
    public static final double MAX_POS = -52; // Max position in encoder units
    public static final double MIN_POS = 0.1; // Min position in encoder units
    public static final double LOAD_POS = -22.5; // TODO: Set this value
    public static final double L1_POS = 0.1; // TODO: Verify this value
    public static final double L2_POS = 0.1; // TODO: Verify this value
    public static final double L3_POS = 0.1; // TODO: Verify this value
    public static final double L4_POS = -50; // TODO: Verify this value
    public static final double P = 0.08; // PID Proportional gain
    public static final double I = 0; // PID Integral gain
    public static final double D = 0; // PID Derivative gain
    public static final double PID_TOLERANCE = 1; // PID tolerance in encoder units
    public static final double KF_BELOW_0_5 = 0.01; // Feedforward below 0.5 units
    public static final double KF_ABOVE_0_5 = 0.025; // Feedforward above 0.5 units
    public static final double THROTTLE_DIVISOR = 65.71; // For drive throttle calculation
}
