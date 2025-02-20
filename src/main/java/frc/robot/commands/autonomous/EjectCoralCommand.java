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

    public EjectCoralCommand() {
        addRequirements(endEffectorSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(armSubsystem);
    }
    
    @Override
    public void initialize() {
        //finished = true;
    }

    @Override
    public void execute() {
        if(endEffectorSubsystem.hasCoral()) {
            ArmState armState = armSubsystem.getDesiredState();

            if(armState == ArmState.CoralL1) {
                this.endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralFront);
            } else if(armState == ArmState.CoralL2) {
                this.endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralFront);
            } else if(armState == ArmState.CoralL3) {
                this.endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralBack);
            } else if(armState == ArmState.CoralL4) {
                this.endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralBack);
            }
            
        } else {
            System.out.println("EjectCoralCommand::execute() - do not have the coral, stopping");
            this.endEffectorSubsystem.setDesiredState(EndEffectorState.Stopped);
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
