package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ReversibleEncoder;

public class LiftSubsystem extends SubsystemBase {
    private final CANSparkMax motor;
    RelativeEncoder encoder;
    SparkPIDController pid;

    private final Servo servo;
    private final boolean reversed;
    private final boolean lockReversed;
    
    private boolean dynamicLiftDisabled = true;
    private double positionRequested = 0;
    private double timePreviousSeconds = System.currentTimeMillis()/1000.0;
    private double velocityPIDPrior = 0;

    public double accelerationRPMperSec = _c.DEFAULT_ACCELERATION_RPM_S;
    public double velocityMax = _c.DEFAULT_VELOCITY_MAX;


    boolean desiredStateIsUnlocked = false;
    public String name;

    private int debugMessageSkipCounter = 0;
    private boolean printDebugMessages;

    public static class _c {
        public static final double DEFAULT_ACCELERATION_RPM_S = 20000.0, DEFAULT_VELOCITY_MAX = 5000.0;
        public static final int ID_L = 12, ID_R = 13, RR_ID_L = 8, RR_ID_R = 9;
        public static final double liftMaxValue = 155.0;
        public static final boolean REVERSED_L = false, REVERSED_R = false, LOCK_REVERSED_L = true, LOCK_REVERSED_R = false;
    }

    @Override
    public void periodic(){


        printDebugMessages = Constants.DEBUG_MODE && debugMessageSkipCounter==0;
        if(debugMessageSkipCounter-- == 0) debugMessageSkipCounter = 300;

        if(!dynamicLiftDisabled){
            boolean liftRatchetUnlocked = ((System.currentTimeMillis() - timeOfLastUnlock) > 1000/3) && desiredStateIsUnlocked;
            double positionCurrent = getPosition();
            double direction = Math.signum(positionRequested - positionCurrent);
            double absPositionError = Math.abs(positionRequested - positionCurrent);

            double timeCurrentSeconds = System.currentTimeMillis()/1000.0;
            double timeDeltaSeconds = timeCurrentSeconds - timePreviousSeconds;
            timePreviousSeconds = timeCurrentSeconds;

            // Zero out our position error if we are inside the deadzone
            absPositionError = absPositionError < 1? 0: absPositionError;
            //// if(Constants.DEBUG_MODE)  SmartDashboard.putNumber(name + "lift position error", absPositionError);

            // If need to lower the robot, unlock the ratchet (increasing position extends the lift, so positive direciton is lowering the robot)
            if(direction > 0.0){
                if(name=="left") if(printDebugMessages) System.out.println("Unlocking " + name + "lift");
                unlock(); 
            }

            // Clamp timeDelta so we don't have too large an acceleration if there was update lag
            if(timeDeltaSeconds > .05) timeDeltaSeconds=.05;
            //// if(Constants.DEBUG_MODE)  SmartDashboard.putNumber(name + "lift timeDelta", timeDeltaSeconds);

            // We want to calcualte a velocity profile to arrive at our destination with constant acceleration to maximum
            // veocity, then constant deceleration to 0 velocity.  We will always assume we are traveling "forward" and fix our
            // negatives at the end using 'direction'.
            
            // Rather than working forward in time, we will work backward in time:  We will "start" at our destination with zero velocity
            // and calculate how fast we would be goimg with constant acceleration "once we arrive" at our current position.
            
            // We will then uniformly increase our actual velocity to match this calculated velocity if we are going slower than it,
            // or increase our actual velocity if we are going faster than it.
           
            // Start with the basic kinematic equation:
            
            // Distance = 1/2 accleration * time^2 (assuming 0 initial velocity, 0 initial position)
            // and solve for time:

            // Time = sqrt(2 Distance / acceelration)

            //   Current   Target
            //   |<-\ time ->|

            // Calculate how long would it take to traverse from the destination to the current position starting from zero velocity
            double timeToTaverseFromZeroVelocity = Math.sqrt(2 * absPositionError / (accelerationRPMperSec/60));
            //// if(Constants.DEBUG_MODE)  SmartDashboard.putNumber(name + "lift travel time from zero velocity", timeToTaverseFromZeroVelocity);

            if(timeToTaverseFromZeroVelocity<1) timeToTaverseFromZeroVelocity = timeToTaverseFromZeroVelocity*timeToTaverseFromZeroVelocity;

            // We will now go back to using positives and negatives to represent direciton since we have to deal with our real
            // motor speed.

            // Using the basic kinematic equation
            // Velocity=Acceleration*Time   (assuming 0 initial velocity, 0 initial position)
            // To figure out what our velocity would be after accelerationg for that period of time
            double velocityDesired = timeToTaverseFromZeroVelocity * accelerationRPMperSec * direction;
            //// if(Constants.DEBUG_MODE)  SmartDashboard.putNumber(name + "lift desired velocity", velocityDesired);

            // Now that we know what our velocity would need to be in order to perfectly decelerate to the desintation, update our
            // current velocity to try to match it. 

            double velocityPIDNew;

            if(velocityDesired >= velocityPIDPrior) { // If we need to increase veloctiy or stay the same speed:

                // Increase our (possibly eitiher negative or positive) current velocity using the kinematic equation:
                // Velocity = Accleration * delta_Time + Initial_Velocity 
                double velocityAfterAccel = accelerationRPMperSec * timeDeltaSeconds + velocityPIDPrior;
                //if(Constants.DEBUG_MODE)  SmartDashboard.putNumber(name + " lift after accel", velocityAfterAccel);

                // Make sure we don't end up faster than our target velocity
                double velocityAfterClampToTaget = velocityAfterAccel > velocityDesired? velocityDesired: velocityAfterAccel;
                //if(Constants.DEBUG_MODE)  SmartDashboard.putNumber(name + " lift after target limit", velocityAfterClampToTaget);                
                
                // Make sure we don't exceed the maximum velocity
                velocityPIDNew = velocityAfterClampToTaget > velocityMax? velocityMax: velocityAfterClampToTaget;
                //if(Constants.DEBUG_MODE)  SmartDashboard.putNumber(name + " lift after vel max", velocityPIDNew);

                // Make sure we don't brownout from going against the ratchet by not allowing positive velocity unless we are safely unlocked
                if(velocityPIDNew > 0 && !liftRatchetUnlocked) velocityPIDNew = 0;

                // If we want to unlock but are not yet unlocked, be sure we are removing pressure from the lock
                if(velocityPIDNew > -200 && desiredStateIsUnlocked && !liftRatchetUnlocked) velocityPIDNew = -200;

                //if(Constants.DEBUG_MODE)  SmartDashboard.putNumber(name + " lift after brownout", velocityPIDNew);
            } 
            else  { // If we need to slow down:
                // Decrease our (possibly eitiher negative or positive) current velocity using the kinematic equation:
                // Velocity = Accleration * delta_Time + Initial_Velocity 
                double velocityAfterAccel =  -accelerationRPMperSec * timeDeltaSeconds + velocityPIDPrior;
                if(Constants.DEBUG_MODE)  SmartDashboard.putNumber(name + " lift after accel", velocityAfterAccel);

                // Make sure we don't end up faster than our target (negative) velocity
                velocityPIDNew = velocityAfterAccel < velocityDesired? velocityDesired: velocityAfterAccel;
                if(Constants.DEBUG_MODE)  SmartDashboard.putNumber(name + " lift after vel max", velocityPIDNew);
                
                // Make sure we don't exceed the maximum velocity
                velocityPIDNew = velocityPIDNew < (-1.0*velocityMax) ? (-1.0*velocityMax): velocityPIDNew;
                if(Constants.DEBUG_MODE)  SmartDashboard.putNumber(name + " lift after brownout", velocityPIDNew);
            }

            // If we are not trying to extend the lift (lower the robot)
            // AND our CURRENT ACTUAL (that is, prior set) velocity is not extending the lift
            // THEN it is safe to lock the lift
            if(direction <= 0.0 && Math.abs(getVelocity()) <= 10) {
                if(name == "left") if(printDebugMessages) System.out.println("Locking " + name + "lift");
                lock();
            } 
            
            //// if(Constants.DEBUG_MODE)  SmartDashboard.putNumber(name + "lift PID velocity ", velocityPIDNew);

            //// if(Constants.DEBUG_MODE)  SmartDashboard.putNumber(name + "left Velocity error ", velocityPIDPrior-this.encoder.getVelocity());
                            

            setVelo(velocityPIDNew);
            velocityPIDPrior=velocityPIDNew;
        }
    }

    public LiftSubsystem(CANSparkMax motor, Servo servo, boolean reversed, boolean lockReversed, String name){
        this.name = name;
        this.motor = motor;
        this.encoder = motor.getEncoder();
        this.pid = motor.getPIDController();
        this.servo = servo;
        this.reversed = reversed;
        this.lockReversed = lockReversed;
    }

    public void setVelo(double speed) {
        pid.setReference(speed * (reversed? -1: 1), CANSparkMax.ControlType.kVelocity);
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public RelativeEncoder getEncoder() {
        return new ReversibleEncoder(encoder, reversed);
    }

    public double getPosition() {
        return encoder.getPosition();
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

    public void set(double output){
        motor.set(output);
        if(Constants.DEBUG_MODE) System.out.print(".MS" + name + ".");
    }
    public void switchToPercentPowerMode(){
        motor.getPIDController().setReference(0, ControlType.kDutyCycle);
        if(Constants.DEBUG_MODE) System.out.println("Switching " +name+" lift to percent power mode" + getID());
    }

    public void setLiftPeriodicDisabled(boolean disabled){
        //if(Constants.DEBUG_MODE) System.out.println(disabled?"Dis":"En"+"able periodic updates for " + name +" lift");
        this.dynamicLiftDisabled = disabled;
        if(disabled) setVelo(0);
    }

    public void setPositionRequested(double positionRequested){
        this.positionRequested = positionRequested;
    }

    public int getID(){
        return this.motor.getDeviceId();
    }
}
