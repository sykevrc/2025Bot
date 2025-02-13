package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class StopIntake extends Command {
    private EndEffectorSubsystem endEffectorSubsystem = RobotContainer.endEffectorSubsystem;

    public StopIntake() {
        addRequirements(endEffectorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        endEffectorSubsystem.setDesiredState(EndEffectorState.Stopped);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
