package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class DynamicDeltaLiftCommand extends Command{
    @SuppressWarnings(value = {"unused"})
    private final DoubleSupplier delta, wantedPosition;
    private final double maxVelocity, acceleration;
    private final LiftSubsystem liftL, liftR;
    private DoubleSupplier supplierL, supplierR;

    public DynamicDeltaLiftCommand(DoubleSupplier delta, DoubleSupplier wantedPosition, double maxVelocity, double acceleration, LiftSubsystem liftL, LiftSubsystem liftR){
        this.delta = delta;
        this.wantedPosition = wantedPosition;
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration;
        this.liftL = liftL;
        this.liftR = liftR;
        
    }

    @Override
    public void initialize(){
        new DynamicLiftCommand(liftL, supplierL, acceleration, 0, maxVelocity);
        new DynamicLiftCommand(liftR, supplierR, acceleration, 0, maxVelocity);
        
        supplierL = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                double leftPos = (wantedPosition.getAsDouble() + delta.getAsDouble()/2), rightPos = (wantedPosition.getAsDouble() - delta.getAsDouble()/2);

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
                if(leftPos * Math.signum(LiftSubsystem._c.RATIO_L) > LiftSubsystem._c.liftMaxValue) leftPos = LiftSubsystem._c.liftMaxValue;
                
                return leftPos;
            }    
        };

        supplierL = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                double leftPos = (wantedPosition.getAsDouble() + delta.getAsDouble()/2), rightPos = (wantedPosition.getAsDouble() - delta.getAsDouble()/2);

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

                if(rightPos * Math.signum(LiftSubsystem._c.RATIO_R) < 0) rightPos = 0;
                if(rightPos * Math.signum(LiftSubsystem._c.RATIO_R) > LiftSubsystem._c.liftMaxValue) rightPos = LiftSubsystem._c.liftMaxValue;
                
                return rightPos;
            }
        };
        
        new DynamicLiftCommand(liftL, supplierL, acceleration, 0, maxVelocity);
        new DynamicLiftCommand(liftR, supplierR, acceleration, 0, maxVelocity);

    }

    @Override
    public void execute(){}

    @Override
    public boolean isFinished(){return true;}
}
