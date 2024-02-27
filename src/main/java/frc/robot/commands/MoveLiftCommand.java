package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LiftSubsystem;

public class MoveLiftCommand extends Command {
    
    private final static double UNDERSHOT_DISTANCE = 0.;

    private DigitalInput SRNSwitchL = Robot.SRNSwitchL;

    double endPos_delta, maxVelo;
    
    private double midPos_delta;

    private double startTime, accel;
    
    private LiftSubsystem lift;

    private RelativeEncoder encoder;

    private enum Stage{
        ACCELERATE, CRUISE, DECELERATE, STOP
    }
    
    private Stage stage;

    private double startPos;

    private double decelPos_delta;
    private double stopPos_delta;

    private int rMult;

    public MoveLiftCommand(LiftSubsystem lift, double position, double maxVelocity, double acceleration){
        addRequirements(lift);
        
        assert(maxVelocity > 0); 
        assert(acceleration > 0);

        
        this.lift = lift;
        this.accel = acceleration;
        this.maxVelo = maxVelocity;
        
        this.encoder = lift.getEncoder();
        this.startTime = System.currentTimeMillis()/1000.0;
        this.startPos = encoder.getPosition();

        double currPos = encoder.getPosition();
        double maxAccelTime = maxVelocity / acceleration;
        
        this.rMult = position < currPos? -1: 1;
        this.endPos_delta = (position - currPos) * rMult;
        this.midPos_delta = (endPos_delta - UNDERSHOT_DISTANCE)/2;
        this.stopPos_delta = endPos_delta - UNDERSHOT_DISTANCE;
        this.decelPos_delta = (endPos_delta - (.5 * accel / 60 * maxAccelTime * maxAccelTime) - UNDERSHOT_DISTANCE);
        
        System.out.println("(.5 * accel_rpmps / 60 * maxAccelTime * maxAccelTime): " + (.5 * accel / 60 * maxAccelTime * maxAccelTime));
        System.out.println("decelPos_delta_rot: " + decelPos_delta);
        System.out.println("current: "+this.startPos+"  midpoint: "+this.midPos_delta+"  dcelPos: "+decelPos_delta+"  stopPos: "+this.stopPos_delta+"   endPos:"+endPos_delta);
        System.out.println("maxAccelTime: " + maxAccelTime);
    }

    @Override
    public void initialize(){
        stage = Stage.ACCELERATE;
        if(this.stopPos_delta <= 0) lift.lock();
        else lift.unlock();
        
        double lockTime_sec = System.currentTimeMillis();
        while(lockTime_sec + 10 < System.currentTimeMillis()){}
    }

    public void execute(){
        double currTime = System.currentTimeMillis()/1000.0;
        double elapTime = currTime - startTime;
        double pos_delta = (encoder.getPosition() - startPos) * rMult; 
        double distRemain = (stopPos_delta - pos_delta);

        
        double newVelo = 0;
        
        System.out.println();
        //System.out.println(stage);
        System.out.println("delta " + pos_delta);
        System.out.println("velo " + newVelo);
        System.out.println("distRemain " + distRemain);

        if(SRNSwitchL.get() == true){
            stage = Stage.STOP;
        }        

        if(stage == Stage.ACCELERATE){
            newVelo = elapTime * accel;
            
            System.out.println("pos_delta_rots" + pos_delta);
            System.out.println("midPos_delta_rot" + midPos_delta);

            if (pos_delta >= midPos_delta) {
                stage = Stage.DECELERATE;
                System.out.println("Switch to DECELERATE");
            }
            else if (newVelo >= maxVelo){
                stage = Stage.CRUISE;
                newVelo = maxVelo;
                System.out.println("Switch to CRUISE");
            }
        }
        
        if(stage == Stage.CRUISE){
            newVelo = maxVelo;
            if (pos_delta >= decelPos_delta) {
                stage = Stage.DECELERATE;
                System.out.println("Switch to DECELERATE");
            }
        }
        
        if(stage == Stage.DECELERATE) {
            System.out.println("distRemain: " + distRemain);
            if (pos_delta >= stopPos_delta) {
                System.out.println("Switch to STOP");
                stage = Stage.STOP;
            }
            else newVelo = Math.sqrt(2.0 * distRemain * accel * 60);
        }

        if(stage == Stage.STOP) newVelo = 0;
        
        lift.setVelo(newVelo * rMult);

    }

    @Override
    public boolean isFinished(){
        return stage == Stage.STOP;
    }

    public boolean waitMillis(int timeoutMillis){
        try{wait(timeoutMillis);}catch (InterruptedException e) {return true;}
        return false;
    }
}
