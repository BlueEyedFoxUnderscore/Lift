package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RatioReversibleEncoder;

public class LiftSubsystem extends SubsystemBase {
    public final CANSparkMax motor;
    RelativeEncoder encoder;
    SparkPIDController pid;
    private final DigitalInput SRNSwitch;
    private final double ratio;
    private final Servo servo;
    private final boolean reversed;

    public static class _c {
        public static final int ID_L = 12, ID_R = 13, SRN_ID_L = 0, SRN_ID_R = 1, RR_ID_L = 8, RR_ID_R = 9;
        public static final double RATIO_L = 1, RATIO_R = 1, liftMaxValue = 325;
    }

    public LiftSubsystem(CANSparkMax motor, DigitalInput SRNSwitch, double ratio, Servo servo, boolean reversed){
        this.motor = motor;
        this.encoder = motor.getEncoder();
        this.pid = motor.getPIDController();
        this.SRNSwitch = SRNSwitch;
        this.ratio = ratio;
        this.servo = servo;
        this.reversed = reversed;
    }

    public void setVelo(double speed) {
        System.out.println("setting speed" + speed * (reversed? -1: 1) * ratio);
        pid.setReference(SRNSwitch.get()? 0: speed * (reversed? -1: 1) * ratio, CANSparkMax.ControlType.kVelocity);
    }

    public void setVelo_override(double speed) throws Exception {
        if(speed > 1750) throw new Exception("Speed too high for override.");
        pid.setReference(speed * (reversed? -1: 1) * ratio , CANSparkMax.ControlType.kVelocity);
    }

    public void setPos(double pos) {
        pid.setReference(pos * ratio, CANSparkMax.ControlType.kPosition);
    }

    public RelativeEncoder getEncoder() {
        return new RatioReversibleEncoder(encoder, ratio, reversed);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public void refresh(){
        encoder = motor.getEncoder();
        pid = motor.getPIDController();
    }

    public void lock(){
        servo.set(1);
        System.out.println("Set to one");
    }

    public void unlock(){
        servo.set(0);
        System.out.println("Set to zre");
    }

    public boolean getSRN(){
        return SRNSwitch.get();
    }
}
