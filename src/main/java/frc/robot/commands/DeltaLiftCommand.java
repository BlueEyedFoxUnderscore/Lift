package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class DeltaLiftCommand extends Command{
    private final double delta, wantedPosition, maxVelocity, acceleration;
    private final LiftSubsystem liftL, liftR;

    public DeltaLiftCommand(double delta, double wantedPosition, double maxVelocity, double acceleration, LiftSubsystem liftL, LiftSubsystem liftR){
        this.delta = delta;
        this.wantedPosition = wantedPosition;
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.liftL = liftL;
        this.liftR = liftR;
    }

    @Override
    public void initialize(){
        double leftPos = (wantedPosition + delta/2), rightPos = (wantedPosition - delta/2);


        if(leftPos * Math.signum(LiftSubsystem._c.RATIO_L) < 0) {
        double reduction = 0 - leftPos;
        leftPos = 0;
        rightPos += reduction;
        }
        
        if(rightPos * Math.signum(LiftSubsystem._c.RATIO_R) < 0){
        double reduction = 0 - rightPos;
        rightPos = 0;
        leftPos += reduction;
        }
        
        if(leftPos * Math.signum(LiftSubsystem._c.RATIO_L) > LiftSubsystem._c.liftMaxValue) {
        double reduction = leftPos - LiftSubsystem._c.liftMaxValue;
        leftPos = LiftSubsystem._c.liftMaxValue;
        rightPos -= reduction;
        }
        
        if(rightPos * Math.signum(LiftSubsystem._c.RATIO_R) > LiftSubsystem._c.liftMaxValue){
        double reduction = rightPos - LiftSubsystem._c.liftMaxValue;
        rightPos = LiftSubsystem._c.liftMaxValue;
        leftPos -= reduction;
        }

        if(leftPos * Math.signum(LiftSubsystem._c.RATIO_L) < 0) leftPos = 0;
        if(rightPos * Math.signum(LiftSubsystem._c.RATIO_R) < 0) rightPos = 0;
        if(leftPos * Math.signum(LiftSubsystem._c.RATIO_L) > LiftSubsystem._c.liftMaxValue) leftPos = LiftSubsystem._c.liftMaxValue;
        if(rightPos * Math.signum(LiftSubsystem._c.RATIO_R) > LiftSubsystem._c.liftMaxValue) rightPos = LiftSubsystem._c.liftMaxValue;
        

        SmartDashboard.putBoolean("Start move", false);
        (new MoveLiftCommand(
        liftL, 
        leftPos, 
        maxVelocity, 
        acceleration
        )).schedule();
        (new MoveLiftCommand(
        liftR, 
        rightPos, 
        maxVelocity, 
        acceleration
        )).schedule();
    }

    @Override
    public void execute(){}

    @Override
    public boolean isFinished(){return true;}
}
