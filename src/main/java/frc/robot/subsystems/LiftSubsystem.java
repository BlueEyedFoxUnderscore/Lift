package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {
    public final CANSparkMax motor;
    RelativeEncoder encoder;
    SparkPIDController pid;
    public final DigitalInput SRNSwitch;
    public final double ratio;

    public static class _c {
        public static final int ID_L = 12, ID_R = 13, SRN_ID_L = 0, SRN_ID_R = 1;
        public static final double RATIO_L = 1, RATIO_R = 20f/50f;
    }

    public LiftSubsystem(CANSparkMax motor, DigitalInput SRNSwitch, double ratio){
        this.motor = motor;
        this.encoder = motor.getEncoder();
        this.pid = motor.getPIDController();
        this.SRNSwitch = SRNSwitch;
        this.ratio = ratio;
    }

    public void setVelo(double speed) {
        System.out.println("setting speed"+speed);
        pid.setReference(SRNSwitch.get()? 0: speed * ratio, CANSparkMax.ControlType.kVelocity);
    }

    public void setVelo_override(double speed) throws Exception {
        if(speed > 1750) throw new Exception("Speed too high for override.");
        pid.setReference(speed * ratio, CANSparkMax.ControlType.kVelocity);
    }

    public void setPos(double pos) {
        pid.setReference(pos * ratio, CANSparkMax.ControlType.kPosition);
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public void refresh(){
        encoder = motor.getEncoder();
        pid = motor.getPIDController();
    }
}
