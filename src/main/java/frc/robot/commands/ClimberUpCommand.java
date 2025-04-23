package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimberUpCommand extends Command {

    
    private ClimbSubsystem climbSubsystem;
    private boolean finished = false;

    public ClimberUpCommand() {
        this.climbSubsystem = RobotContainer.climbSubsystem;
        addRequirements(climbSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finished = true;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (Constants.ClimbConstants.climberEnabled){
            climbSubsystem.setState(ClimbSubsystem.ClimberState.EXTENDED);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}