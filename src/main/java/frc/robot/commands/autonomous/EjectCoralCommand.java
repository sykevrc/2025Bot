package frc.robot.commands.autonomous;

import java.util.OptionalLong;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EjectCoralCommand extends Command {

    private EndEffectorSubsystem endEffectorSubsystem = RobotContainer.endEffectorSubsystem;
    private boolean finished = false;
    private OptionalLong ejectTime = OptionalLong.empty();

    public EjectCoralCommand(OptionalLong ejectTime) {
        addRequirements(endEffectorSubsystem);

        this.ejectTime = ejectTime;
    }
    
    @Override
    public void initialize() {
         if(ejectTime.isPresent()) {

            TimerTask task = new TimerTask() {
                public void run() {
                    System.out.println("stopping the eject");
                    finished = true;
                }
            };
            Timer timer = new Timer("Timer");
    
            timer.schedule(task, ejectTime.getAsLong());
            System.out.println("starting the eject");
            finished = false;
        } else {
            finished = true;
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
