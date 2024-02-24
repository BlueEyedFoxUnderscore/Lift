package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LiftSubsystem;

public class MoveLiftCommand extends Command {
    
    private final static double UNDERSHOT_DISTANCE = 0.;

    private DigitalInput theStopRightNowSwitch = Robot.theStopRightNowSwitch;

    double endPos_delta_rots, maxVelo_rpm;
    
    private double midPos_delta_rot;

    private double startTime_sec, accel_rpmps;
    
    private LiftSubsystem lift;

    private RelativeEncoder encoder;

    private enum Stage{
        ACCELERATE, CRUISE, DECELERATE, STOP
    }
    
    private Stage stage;

    private double startPos_rot;

    private double decelPos_delta_rot;
    private double stopPos_delta_rot;

    private int rMult;

    public MoveLiftCommand(LiftSubsystem lift, double position, double maxVelocity, double acceleration){
        addRequirements(lift);
        
        assert(maxVelocity > 0); 
        assert(acceleration > 0);

        
        this.lift = lift;
        this.accel_rpmps = acceleration;
        this.maxVelo_rpm = maxVelocity;
        
        this.encoder = lift.getEncoder();
        this.startTime_sec = System.currentTimeMillis()/1000.0;
        this.startPos_rot = encoder.getPosition();

        double currPos_rots = encoder.getPosition();
        double maxAccelTime = maxVelocity / acceleration;
        
        this.rMult = position < currPos_rots? -1: 1;
        this.endPos_delta_rots = (position - currPos_rots) * rMult;
        this.midPos_delta_rot = (endPos_delta_rots - UNDERSHOT_DISTANCE)/2;
        this.stopPos_delta_rot = endPos_delta_rots - UNDERSHOT_DISTANCE;
        this.decelPos_delta_rot = (endPos_delta_rots - (.5 * accel_rpmps / 60 * maxAccelTime * maxAccelTime) - UNDERSHOT_DISTANCE);
        
        System.out.println("(.5 * accel_rpmps / 60 * maxAccelTime * maxAccelTime): " + (.5 * accel_rpmps / 60 * maxAccelTime * maxAccelTime));
        System.out.println("decelPos_delta_rot: " + decelPos_delta_rot);
        System.out.println("current: "+this.startPos_rot+"  midpoint: "+this.midPos_delta_rot+"  dcelPos: "+decelPos_delta_rot+"  stopPos: "+this.stopPos_delta_rot+"   endPos:"+endPos_delta_rots);
        System.out.println("maxAccelTime: " + maxAccelTime);
    }

    @Override
    public void initialize(){
        stage = Stage.ACCELERATE;
    }

    public void execute(){
        double currTime_secs = System.currentTimeMillis()/1000.0;
        double elapTime_secs = currTime_secs - startTime_sec;
        double pos_delta_rots = (encoder.getPosition() - startPos_rot) * rMult; 
        double distRemain_rot = (stopPos_delta_rot - pos_delta_rots);

        
        double newVelo_rpm = 0;
        
        System.out.println();
        //System.out.println(stage);
        System.out.println("delta " + pos_delta_rots);
        System.out.println("velo " + newVelo_rpm);
        System.out.println("distRemain " + distRemain_rot);

        if(theStopRightNowSwitch.get() == true){
            stage = Stage.STOP;
        }        

        if(stage == Stage.ACCELERATE){
            newVelo_rpm = elapTime_secs * accel_rpmps;
            
            System.out.println("pos_delta_rots" + pos_delta_rots);
            System.out.println("midPos_delta_rot" + midPos_delta_rot);

            if (pos_delta_rots >= midPos_delta_rot) {
                stage = Stage.DECELERATE;
                System.out.println("Switch to DECELERATE");
            }
            else if (newVelo_rpm >= maxVelo_rpm){
                stage = Stage.CRUISE;
                newVelo_rpm = maxVelo_rpm;
                System.out.println("Switch to CRUISE");
            }
        }
        
        if(stage == Stage.CRUISE){
            newVelo_rpm = maxVelo_rpm;
            if (pos_delta_rots >= decelPos_delta_rot) {
                stage = Stage.DECELERATE;
                System.out.println("Switch to DECELERATE");
            }
        }
        
        if(stage == Stage.DECELERATE) {
            System.out.println("distRemain: " + distRemain_rot);
            if (pos_delta_rots >= stopPos_delta_rot) {
                System.out.println("Switch to STOP");
                stage = Stage.STOP;
            }
            else newVelo_rpm = Math.sqrt(2.0 * distRemain_rot * accel_rpmps * 60);
        }

        if(stage == Stage.STOP) newVelo_rpm = 0;
        
        lift.setVelo(newVelo_rpm * rMult);

    }

    @Override
    public boolean isFinished(){
        return stage == Stage.STOP;
    }
}
