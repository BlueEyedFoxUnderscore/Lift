package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LiftSubsystem;

public class ZeroLiftCommand extends Command{
    private LiftSubsystem lift = new LiftSubsystem();
    private State stateL;
    private boolean isFinishedL;
    private State stateR;
    private boolean isFinishedR;
    public ZeroLiftCommand(LiftSubsystem subsystem){
        this.lift = subsystem;
    }
    
    private static enum State {
        DOWN_QUICK, UP_QUICK, DOWN_SLOW, UP_SLOW, STOP
    }

    @Override
    public void initialize(){
        addRequirements(lift);
        stateL = State.DOWN_QUICK;
    }

    @Override
    public void execute(){
        cycleL();
        cycleR();
    }

    @Override
    public boolean isFinished(){
        return this.isFinishedL && isFinishedR;
    }

    public void cycleL(){
        switch(stateL){
            case DOWN_QUICK:
                lift.setVelo_l(-1750);
                if(Robot.SRNSwitchL.get()) stateL = State.UP_QUICK;
                System.out.println("dQ");
                break;
            case UP_QUICK:
                lift.setVelo_l(1750);
                if(!Robot.SRNSwitchL.get()) stateL = State.DOWN_SLOW;
                System.out.println("uQ");
                break;
            case DOWN_SLOW:
                lift.setVelo_l(-100);
                if(Robot.SRNSwitchL.get()) stateL = State.UP_SLOW;
                System.out.println("dS");
                break;
            case UP_SLOW:
                lift.setVelo_l(100);
                if(!Robot.SRNSwitchL.get()) stateL = State.STOP;
                System.out.println("uS");
                break;
            case STOP:
                System.out.println("S");
                lift.setVelo_l(0);
                this.isFinishedL = true;
                lift.getEncoderL().setPosition(0);
                break;
        }
    }

    public void cycleR(){
        switch(stateR){
            case DOWN_QUICK:
                lift.setVelo_r(-1750);
                if(Robot.SRNSwitchR.get()) stateR = State.UP_QUICK;
                System.out.println("dQ");
                break;
            case UP_QUICK:
                lift.setVelo_r(1750);
                if(!Robot.SRNSwitchR.get()) stateR = State.DOWN_SLOW;
                System.out.println("uQ");
                break;
            case DOWN_SLOW:
                lift.setVelo_r(-100);
                if(Robot.SRNSwitchR.get()) stateR = State.UP_SLOW;
                System.out.println("dS");
                break;
            case UP_SLOW:
                lift.setVelo_r(100);
                if(!Robot.SRNSwitchR.get()) stateR = State.STOP;
                System.out.println("uS");
                break;
            case STOP:
                System.out.println("S");
                lift.setVelo_r(0);
                this.isFinishedR = true;
                lift.getEncoderR().setPosition(0);
                break;
        }
    }

}
