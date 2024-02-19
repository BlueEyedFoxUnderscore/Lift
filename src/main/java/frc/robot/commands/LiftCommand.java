package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class LiftCommand extends Command {
    public enum Commands {
        UP, DOWN
    }

    private LiftSubsystem lift;

    public LiftCommand(LiftSubsystem subsystem, DoubleSupplier position){
        lift = subsystem;
    }

    @Override
    public void initialize(){
        addRequirements(lift);
        
        SmartDashboard.putNumber("Wanted Position", 0);
    }

    public void execute(){

        double wPos = SmartDashboard.getNumber("Wanted Position", 0);
        System.out.println("LiftCommand.execute: Set velocity");
        lift.setVelo(wPos);
        System.out.println(wPos);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
