package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LiftSubsystem;

public class ZeroLiftCommand extends Command{
    private LiftSubsystem lift = new LiftSubsystem();
    private State state;
    private boolean isFinished;
    public ZeroLiftCommand(LiftSubsystem subsystem){
        this.lift = subsystem;
    }
    
    private static enum State {
        DOWN_QUICK, UP_QUICK, DOWN_SLOW, UP_SLOW, STOP
    }

    @Override
    public void initialize(){
        addRequirements(lift);
        state = State.DOWN_QUICK;
    }

    @Override
    public void execute(){
        switch(state){
            case DOWN_QUICK:
                lift.setVelo(-400);
                if(Robot.theStopRightNowSwitch.get()) state = State.UP_QUICK;
                System.out.println("dQ");
                break;
            case UP_QUICK:
                lift.setVelo(400);
                if(!Robot.theStopRightNowSwitch.get()) state = State.DOWN_SLOW;
                System.out.println("uQ");
                break;
            case DOWN_SLOW:
                lift.setVelo(-100);
                if(Robot.theStopRightNowSwitch.get()) state = State.UP_SLOW;
                System.out.println("dS");
                break;
            case UP_SLOW:
                lift.setVelo(100);
                if(!Robot.theStopRightNowSwitch.get()) state = State.STOP;
                System.out.println("uS");
                break;
            case STOP:
                System.out.println("S");
                lift.setVelo(0);
                this.isFinished = true;
                lift.getEncoder().setPosition(0);
                break;
        }
    }

    @Override
    public boolean isFinished(){
        return this.isFinished;
    }
}
