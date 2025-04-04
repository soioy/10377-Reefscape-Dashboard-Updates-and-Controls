package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmIOReal implements ArmIO {
    private final TalonFX armMotor = new TalonFX(ArmConstants.ARM_MOTOR_ID);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.ENCODER_PORT, 1, -0.42);
    private final DigitalInput limitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH_PORT);

    public ArmIOReal() {
        // No specific motor configuration needed for TalonFX in this case

        encoder.setInverted(true);
    }

    @Override
    public void setVoltage(double volts) {
        armMotor.setVoltage(volts);
    }

    @Override
    public double getPosition() {
        // double rawValue = armMotor.getPosition().getValueAsDouble();
        double rawValue = encoder.get();
        return rawValue;
    }

    @Override
    public double getVelocity() {
        return armMotor.getVelocity().getValueAsDouble(); // TalonFX provides velocity in rotations per second
    }

    @Override
    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.position = getPosition();
        inputs.velocity = getVelocity();
        inputs.appliedVolts = armMotor.getMotorVoltage().getValueAsDouble();
        inputs.limitSwitch = getLimitSwitch();
    }

    @Override
    public double setPosition(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }
}
