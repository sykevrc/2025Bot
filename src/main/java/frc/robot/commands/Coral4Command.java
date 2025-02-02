package frc.robot.commands;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SliderSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.SliderSubsystem.SliderState;

public class Coral4Command extends Command {

    SliderSubsystem sliderSubsystem;
    ArmSubsystem armSubsystem;

    public Coral4Command() {
        this.sliderSubsystem = RobotContainer.sliderSubsystem;
        this.armSubsystem = RobotContainer.armSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(sliderSubsystem);
        addRequirements(armSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sliderSubsystem.setDesiredState(SliderState.CoralL4);
    armSubsystem.setDesiredState(ArmState.CoralL4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
