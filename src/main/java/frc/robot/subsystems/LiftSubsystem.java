package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RatioReversibleEncoder;

public class LiftSubsystem extends SubsystemBase {
    public final CANSparkMax motor;
    RelativeEncoder encoder;
    SparkPIDController pid;
    private final DigitalInput SRNSwitch;
    private final double ratio;
    private final Servo servo;
    private final boolean reversed;
    private final int id;
    private final boolean lockReversed;


    public static class _c {
        public static final int ID_L = 12, ID_R = 13, SRN_ID_L = 0, SRN_ID_R = 1, RR_ID_L = 8, RR_ID_R = 9;
        public static final double RATIO_L = 1, RATIO_R = 1, liftMaxValue = 325;
        public static final boolean REVERSED_L = false, REVERSED_R = false, LOCK_REVERSED_L = true, LOCK_REVERSED_R = false;
    }

    public LiftSubsystem(CANSparkMax motor, DigitalInput SRNSwitch, double ratio, Servo servo, boolean reversed, boolean lockReversed){
        this.motor = motor;
        this.encoder = motor.getEncoder();
        this.pid = motor.getPIDController();
        this.SRNSwitch = SRNSwitch;
        this.ratio = ratio;
        this.servo = servo;
        this.reversed = reversed;
        this.id = motor.getDeviceId();
        this.lockReversed = lockReversed;
    }

    public void setVelo(double speed) {
        pid.setReference(SRNSwitch.get()? 0: speed * (reversed? -1: 1) * ratio, CANSparkMax.ControlType.kVelocity);
        if(Constants.DEBUG_MODE) System.out.println("Set speed for ID " + id + " to " + speed * (reversed? -1: 1) * ratio);
    }

    public void setVelo_override(double speed) throws Exception {
        if(speed > 1750) throw new Exception("Speed too high for override.");
        pid.setReference(speed * (reversed? -1: 1) * ratio , CANSparkMax.ControlType.kVelocity);
    }

    public void setPos(double pos) {
        pid.setReference(pos * ratio, CANSparkMax.ControlType.kPosition);
        if(Constants.DEBUG_MODE) System.out.println("Set position for ID " + id + " to " + pos);
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
        if(!lockReversed) servo.set(1);
        else servo.set(0);
        if(Constants.DEBUG_MODE) System.out.println("Locked lift with CAN ID " + id);
    }

    public void unlock(){
        if(!lockReversed) servo.set(0);
        else servo.set(1);
        if(Constants.DEBUG_MODE) System.out.println("Unlocked lift with CAN ID " + id);
    }

    public boolean getSRN(){
        return SRNSwitch.get();
    }

    public void setMaxOutput(double max){
        motor.getPIDController().setOutputRange(max, -max);
    }

    public void set(double output){
        motor.set(output);
        if(Constants.DEBUG_MODE) System.out.println("Set output for ID " + id + " to " + output);
    }

    public void disablePid(){
        motor.getPIDController().setReference(0, ControlType.kDutyCycle);
        if(Constants.DEBUG_MODE) System.out.println("Disabled PID for ID " + id);
    }
}
