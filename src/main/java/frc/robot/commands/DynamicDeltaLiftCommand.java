package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class DynamicDeltaLiftCommand extends Command{
    private final DoubleSupplier delta, wantedPosition;
    private final LiftSubsystem liftL, liftR;

    public DynamicDeltaLiftCommand(DoubleSupplier delta, DoubleSupplier wantedPosition, LiftSubsystem liftL, LiftSubsystem liftR){
        this.delta = delta;
        this.wantedPosition = wantedPosition;
        this.liftL = liftL;
        this.liftR = liftR;
    }

    @Override
    public void initialize(){
        addRequirements(liftL, liftR);
    }

    public void execute(){
        liftL.setPositionRequested(getLeft());
        liftR.setPositionRequested(getRight());
    }
    
    private double getLeft(){
        double leftPos = (wantedPosition.getAsDouble() + delta.getAsDouble()/2), rightPos = (wantedPosition.getAsDouble() - delta.getAsDouble()/2);

        if(leftPos < 0) {
            double reduction = 0 - leftPos;
            leftPos = 0;
            rightPos += reduction;
        }
        
        if(rightPos < 0){
            double reduction = 0 - rightPos;
            rightPos = 0;
            leftPos += reduction;
        }
        
        if(leftPos > LiftSubsystem._c.liftMaxValue) {
            double reduction = leftPos - LiftSubsystem._c.liftMaxValue;
            leftPos = LiftSubsystem._c.liftMaxValue;
            rightPos -= reduction;
        }
        
        if(rightPos > LiftSubsystem._c.liftMaxValue){
            double reduction = rightPos - LiftSubsystem._c.liftMaxValue;
            rightPos = LiftSubsystem._c.liftMaxValue;
            leftPos -= reduction;
        }

        if(leftPos < 0) leftPos = 0;
        if(leftPos > LiftSubsystem._c.liftMaxValue) leftPos = LiftSubsystem._c.liftMaxValue;
        
        return leftPos;
    }

    private double getRight() {
        double leftPos = (wantedPosition.getAsDouble() + delta.getAsDouble()/2), rightPos = (wantedPosition.getAsDouble() - delta.getAsDouble()/2);

        if(leftPos < 0) {
            double reduction = 0 - leftPos;
            leftPos = 0;
            rightPos += reduction;
        }
        
        if(rightPos < 0){
            double reduction = 0 - rightPos;
            rightPos = 0;
            leftPos += reduction;
        }
        
        if(leftPos > LiftSubsystem._c.liftMaxValue) {
            double reduction = leftPos - LiftSubsystem._c.liftMaxValue;
            leftPos = LiftSubsystem._c.liftMaxValue;
            rightPos -= reduction;
        }
        
        if(rightPos > LiftSubsystem._c.liftMaxValue){
            double reduction = rightPos - LiftSubsystem._c.liftMaxValue;
            rightPos = LiftSubsystem._c.liftMaxValue;
            leftPos -= reduction;
        }

        if(rightPos < 0) rightPos = 0;
        if(rightPos > LiftSubsystem._c.liftMaxValue) rightPos = LiftSubsystem._c.liftMaxValue;
        
        return rightPos;
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
