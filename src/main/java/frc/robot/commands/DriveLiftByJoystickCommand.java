package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class DriveLiftByJoystickCommand extends Command{
    private final DoubleSupplier commandedTilt, commmandedLift;
    private final LiftSubsystem liftL, liftR;

    public DriveLiftByJoystickCommand(DoubleSupplier Tilt, DoubleSupplier Lift, LiftSubsystem liftL, LiftSubsystem liftR){
        this.commandedTilt = Tilt;
        this.commmandedLift = Lift;
        this.liftL = liftL;
        this.liftR = liftR;
    }

    @Override
    public void initialize(){
        addRequirements(liftL, liftR);
    }

    public void execute(){
        double leftPos = (commmandedLift.getAsDouble() + commandedTilt.getAsDouble()/2);
        double rightPos = (commmandedLift.getAsDouble() - commandedTilt.getAsDouble()/2);

        double correction;
        if(leftPos < 0) {
            correction = -rightPos;
            leftPos += correction;
            rightPos += correction;
        }
        
        if(rightPos < 0){
            correction = -rightPos;
            leftPos += correction;
            rightPos += correction;
        }
        if(leftPos > LiftSubsystem._c.liftMaxValue) {
            correction = leftPos - LiftSubsystem._c.liftMaxValue;
            leftPos -= correction;
            rightPos -= correction;
        }
        if(rightPos > LiftSubsystem._c.liftMaxValue) {
            correction = rightPos - LiftSubsystem._c.liftMaxValue;
            leftPos -= correction;
            rightPos -= correction;
        }
        
        // If everthing is right, we should not have any values outside our maximums and minimums at this point

        liftL.setPositionRequested(leftPos);
        liftR.setPositionRequested(rightPos);

        //// SmartDashboard.putNumber("commanded Lift", commmandedLift.getAsDouble());
        //// SmartDashboard.putNumber("commanded Tilt", commandedTilt.getAsDouble());
        //// SmartDashboard.putNumber("Left position", leftPos);
        //// SmartDashboard.putNumber("Right position", rightPos);

    }
    
    @Override
    public boolean isFinished(){
        return false;
    }
}
