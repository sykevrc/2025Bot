package frc.robot.commands.autonomous;

import java.util.OptionalLong;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class EjectCoralCommand extends Command {

    private EndEffectorSubsystem endEffectorSubsystem = RobotContainer.endEffectorSubsystem;
    private ElevatorSubsystem elevatorSubsystem = RobotContainer.elevatorSubsystem;
    private ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
    private boolean finished = false;
    private OptionalLong ejectTime = OptionalLong.empty();

    public EjectCoralCommand(OptionalLong ejectTime) {
        addRequirements(endEffectorSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(armSubsystem);

        this.ejectTime = ejectTime;
    }
    
    @Override
    public void initialize() {
         if(ejectTime.isPresent()) {

            TimerTask task = new TimerTask() {
                public void run() {
                    System.out.println("stopping the eject");
                    endEffectorSubsystem.setDesiredState(EndEffectorState.Stopped);
                    elevatorSubsystem.setDesiredState(ElevatorState.CoralHuman);
                    armSubsystem.setDesiredState(ArmState.CoralHuman);
                    finished = true;
                }
            };
            Timer timer = new Timer("Timer");
    
            timer.schedule(task, ejectTime.getAsLong());
            System.out.println("starting the eject");
            endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoral);
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
