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
    private final CANSparkMax motor;
    RelativeEncoder encoder;
    SparkPIDController pid;
    private final DigitalInput SRNSwitch;
    private final double ratio;
    private final Servo servo;
    private final boolean reversed;
    private final boolean lockReversed;
    
    private boolean dynamicLiftDisabled = true;
    private double positionRequested = 0;
    private double timePrevious = System.currentTimeMillis();
    private double velocityLastSet = 0;

    public double acceleration = _c.DEFAULT_ACCELERATION;
    public double velocityMax = _c.DEFAULT_VELOCITY_MAX;

    boolean desiredStateIsUnlocked = false;

    public static class _c {
        public static final double DEFAULT_ACCELERATION = 20000, DEFAULT_VELOCITY_MAX = 5000;
        public static final int ID_L = 12, ID_R = 13, SRN_ID_L = 0, SRN_ID_R = 1, RR_ID_L = 8, RR_ID_R = 9;
        public static final double RATIO_L = 1, RATIO_R = 1, liftMaxValue = 325;
        public static final boolean REVERSED_L = false, REVERSED_R = false, LOCK_REVERSED_L = true, LOCK_REVERSED_R = false;
    }

    @Override
    public void periodic(){
        if(!dynamicLiftDisabled){
            boolean liftUnlocked = ((System.currentTimeMillis() - timeOfLastUnlock) > 500) && desiredStateIsUnlocked;
            double positionCurrent = getPosition();
            double positionError = positionRequested - positionCurrent;
            double timeCurrent = System.currentTimeMillis();
            double timeDelta = timePrevious - timeCurrent;
            
            // Zero error if inside deadzone
            positionError = positionError < 10? 0: positionError;
            
            // Clamp timeDelta so we don't have too large an acceleration
            if (timeDelta > .05) timeDelta=.05;

            timePrevious = timeCurrent;
            // By symetry, the profile we want to get to our destiation is a mirror-image of the profile that we would use to get from
            // our destination to our current position.

            // Calculate how long would it take to traverse from the destination to the current position starting from zero velocity
            double timeToTaverseFromZeroVelocity = Math.sqrt(2 * Math.abs(positionError) / acceleration);

            // If need to lower the lift, unlock
            if(positionError > 0) unlock();

            // Calculate the velocity we would be at after acceleration for that time
            double velocityDesired = timeToTaverseFromZeroVelocity * acceleration * Math.signum(positionError);

            // If we need to speed up or stay the same:
            if (velocityDesired >= velocityLastSet) {
                // Update our velocity to be closer to the ideal velocity 
                velocityLastSet = velocityLastSet + acceleration * timeDelta;

                // Make sure we don't end up faster than our target velocity
                velocityLastSet = velocityLastSet > velocityDesired? velocityDesired: velocityLastSet;
                
                // Make sure we don't exceed the maximum velocity
                velocityLastSet = velocityLastSet > velocityMax? velocityMax: velocityLastSet;

                // Make sure we don't brownout from going against the ratchet
                if(velocityLastSet > 0 && !liftUnlocked) velocityLastSet = 0;
            } 
            // If we need to slow down:
            else {
                // Update our velocity to be closer to the ideal velocity 
                velocityLastSet = velocityLastSet - acceleration * timeDelta;

                // Make sure we don't end up faster than our target (negative) velocity
                velocityLastSet = velocityLastSet < velocityDesired? velocityDesired: velocityLastSet;
                
                // Make sure we don't exceed the maximum velocity
                velocityLastSet = velocityLastSet < velocityMax? velocityMax: velocityLastSet;
            }

            // If we are not lowering the robot, lock
            if(positionError <= 0 && velocityLastSet <= 0) lock();
            
            setVelo(velocityLastSet);
        }
    }

    public LiftSubsystem(CANSparkMax motor, DigitalInput SRNSwitch, double ratio, Servo servo, boolean reversed, boolean lockReversed){
        this.motor = motor;
        this.encoder = motor.getEncoder();
        this.pid = motor.getPIDController();
        this.SRNSwitch = SRNSwitch;
        this.ratio = ratio;
        this.servo = servo;
        this.reversed = reversed;
        this.lockReversed = lockReversed;
    }

    public void setVelo(double speed) {
        pid.setReference(SRNSwitch.get()? 0: speed * (reversed? -1: 1) * ratio, CANSparkMax.ControlType.kVelocity);
        if(Constants.DEBUG_MODE) System.out.println("Set speed for ID " + getID() + " to " + speed * (reversed? -1: 1) * ratio);
    }

    public void setVelo_override(double speed) throws Exception {
        if(speed > 1750) throw new Exception("Speed too high for override.");
        pid.setReference(speed * (reversed? -1: 1) * ratio , CANSparkMax.ControlType.kVelocity);
    }

    @Deprecated
    public void setPosPID(double pos) {
        pid.setReference(pos * ratio, CANSparkMax.ControlType.kPosition);
        if(Constants.DEBUG_MODE) System.out.println("Set position for ID " + getID() + " to " + pos);
    }

    public double getVelocity() {
        return encoder.getVelocity();
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
        if(desiredStateIsUnlocked || firstServoCommand){
            if(!lockReversed) servo.set(1);
            else servo.set(0);
            if(Constants.DEBUG_MODE) System.out.println("Locked lift with CAN ID " + getID());
            desiredStateIsUnlocked = false;
            firstServoCommand = false;
        }
    }

    double timeOfLastUnlock = Double.NaN;
    private boolean firstServoCommand = true;

    public void unlock(){
        if(!desiredStateIsUnlocked || firstServoCommand){
            if(!lockReversed) servo.set(0);
            else servo.set(1);
            if(Constants.DEBUG_MODE) System.out.println("Unlocked lift with CAN ID " + getID());
            timeOfLastUnlock = System.currentTimeMillis();
            desiredStateIsUnlocked = true;
            firstServoCommand = false;
        }
    }

    public boolean getSRN(){
        return SRNSwitch.get();
    }

    public void setMaxOutput(double max){
        motor.getPIDController().setOutputRange(max, -max);
    }

    public void set(double output){
        motor.set(output);
        if(Constants.DEBUG_MODE) System.out.println("Set output for ID " + getID() + " to " + output);
    }

    public void disablePid(){
        motor.getPIDController().setReference(0, ControlType.kDutyCycle);
        if(Constants.DEBUG_MODE) System.out.println("Disabled PID for ID " + getID());
    }

    public void setDynamicLiftDisabled(boolean disabled){
        this.dynamicLiftDisabled = disabled;
        if(disabled) setVelo(0);
    }

    public void setPositionRequested(double positionRequested){
        this.positionRequested = positionRequested;
        setDynamicLiftDisabled(false);
    }

    public int getID(){
        return this.motor.getDeviceId();
    }
}
