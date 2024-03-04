package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LiftSubsystem;

public class ZeroLiftCommand extends Command{
    private LiftSubsystem lift;
    private State state;
    private boolean isFinished;
    public ZeroLiftCommand(LiftSubsystem subsystem){
        this.lift = subsystem;
        lift.refresh();
    }
    
    private static enum State {
        DOWN_QUICK, UP_QUICK, DOWN_SLOW, UP_SLOW, STOP
    }

    @Override
    public void initialize(){
        lift.lock();
        addRequirements(lift);
        setState(State.DOWN_QUICK);
    }

    @Override
    public void execute(){
        switch(state){
            case DOWN_QUICK:
                lift.lock();
                lift.setVelo(-1750);
                if(lift.getSRN()) setState(State.UP_QUICK);
                break;
            case UP_QUICK:
                lift.unlock();
                try {
                    lift.setVelo_override(1750);
                } catch (Exception e) {
                    e.printStackTrace();
                }
                if(!lift.getSRN()) setState(State.DOWN_SLOW);
                break;
            case DOWN_SLOW:
                lift.lock();
                lift.setVelo(-100);
                if(lift.getSRN()) setState(State.UP_SLOW);
                break;
            case UP_SLOW:
                lift.unlock();
                try {
                    lift.setVelo_override(100);
                } catch (Exception e) {
                    e.printStackTrace();
                }
                if(!lift.getSRN()) setState(State.STOP);
                break;
            case STOP:
                lift.setVelo(0);
                this.isFinished = true;
                lift.getEncoder().setPosition(-10);
                break;
        }
    }

    @Override
    public boolean isFinished(){
        return this.isFinished;
    }

    private void setState(State state){
        if(Constants.DEBUG_MODE) System.out.println(this.getName() + "/" + (lift.motor.getDeviceId() == 12? "LEFT": "RIGHT") + ": Set state to " + state);
        this.state = state;
    }
}
