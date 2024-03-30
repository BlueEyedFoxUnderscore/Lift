package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LiftSubsystem;

public class ZeroLiftCommand extends Command{
    private LiftSubsystem lift;
    private boolean isFinished;
    private double timeUnlockStartedMiliseconds;
    private State state = State.UNLOCK;

    private enum State {
        UNLOCK, UNLOCKING, SET_POWER, TRAVEL, WAIT_FOR_SPINUP
    }

    public ZeroLiftCommand(LiftSubsystem subsystem){
        this.lift = subsystem;
    }

    @Override
    public void initialize(){
        //! BUG (Either do it here or in the state machine, not both)
        //// lift.unlock();
        
        addRequirements(lift);
    }

    @Override
    public void execute(){
        switch(state){
            case UNLOCK:
                if(Constants.DEBUG_MODE) System.out.println("Unlocking "+lift.name+" lift");
                timeUnlockStartedMiliseconds = System.currentTimeMillis();
                lift.switchToPercentPowerMode();
                lift.setLiftPeriodicDisabled(true);
                lift.unlock();
                state = State.UNLOCKING;
                break;
            case UNLOCKING:
                //// if(System.currentTimeMillis() < sttime + 500) state = State.SET_POWER; // BUG
                if(System.currentTimeMillis() > timeUnlockStartedMiliseconds + 500) state = State.SET_POWER;
                break;
            case SET_POWER:
                 if(Constants.DEBUG_MODE) System.out.println("Lowering "+lift.name+" lift");
                lift.set(-.2);
                state = State.WAIT_FOR_SPINUP;
                break;
            case WAIT_FOR_SPINUP:
                //// if(System.currentTimeMillis() < sttime + 1500) state = State.TRAVEL; // BUG
                if(System.currentTimeMillis() > timeUnlockStartedMiliseconds + 1500) state = State.TRAVEL;
                break;
            case TRAVEL:
                if(lift.getVelocity() > -400.0){
                    if(Constants.DEBUG_MODE) System.out.println("Reached bottom of "+lift.name+" lift");
                    if(Constants.DEBUG_MODE) System.out.println("Setting encoder position to -2 "+lift.name+" lift");
                    lift.getEncoder().setPosition(-3);
                    if(Constants.DEBUG_MODE) System.out.println("Setting requested position to 0 "+lift.name+" lift");
                    lift.setPositionRequested(0);
                    lift.setLiftPeriodicDisabled(false);

                    isFinished = true;
                }
                break;
        }
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }
}
