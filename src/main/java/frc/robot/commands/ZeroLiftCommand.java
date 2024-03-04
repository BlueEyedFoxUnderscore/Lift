package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LiftSubsystem;

public class ZeroLiftCommand extends Command{
    private LiftSubsystem lift;
    private boolean isFinished;
    private double sttime;
    private State state = State.UNLOCK;

    private enum State {
        UNLOCK, UNLOCKING, SET_POWER, TRAVEL, WAIT_FOR_SPINUP
    }

    public ZeroLiftCommand(LiftSubsystem subsystem){
        this.lift = subsystem;
        lift.refresh();
    }

    @Override
    public void initialize(){
        lift.disablePid();
        lift.setDynamicLiftDisabled(true);
        lift.unlock();
        addRequirements(lift);
        sttime = System.currentTimeMillis();
    }

    @Override
    public void execute(){
        switch(state){
            case UNLOCK:
                lift.unlock();
                state = State.UNLOCKING;
                break;
            case UNLOCKING:
                if (System.currentTimeMillis() < sttime + 500) state = State.SET_POWER;
                break;
            case SET_POWER:
                lift.set(-.1);
                state = State.WAIT_FOR_SPINUP;
                break;
            case WAIT_FOR_SPINUP:
                if (System.currentTimeMillis() < sttime + 1500) state = State.TRAVEL;
                break;
            case TRAVEL:
                if(lift.getVelocity() > -10.0){
                    lift.getEncoder().setPosition(-10);
                    lift.setPositionRequested(0);
                    isFinished = true;
                    if(Constants.DEBUG_MODE) System.out.println("Reached bottom of lift for the " + (lift.getID() == 12? "right": "left") + " lift");
                }
                break;
        }
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }
}
