package frc.robot.commands;

import java.util.OptionalLong;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class StopEjectCoralCommand extends Command {

    private EndEffectorSubsystem endEffectorSubsystem = RobotContainer.endEffectorSubsystem;
    
    public StopEjectCoralCommand() {

        addRequirements(endEffectorSubsystem);

    }

    @Override
    public void initialize() {
         endEffectorSubsystem.setDesiredState(EndEffectorState.EjectCoralFront);
    }

    @Override
    public void execute() {
        this.endEffectorSubsystem.setDesiredState(EndEffectorState.Stopped);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
