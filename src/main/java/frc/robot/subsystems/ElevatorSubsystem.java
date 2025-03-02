package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.Map;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    public enum ElevatorState {
        Start,
        CoralHuman,
        CoralL4,
        CoralL3,
        CoralL2,
        CoralL1,
        AlgaeHuman,
        AlgaeL3,
        AlgaeL2,
        AlgaeL1,
        AlgaeShoot,
        AlgaeFloor
    }

private boolean isSim = false;
private final TalonFX m_Kraken;
private final DutyCycleOut m_DriverOutput;
private double currentPosition = 0.0;
private double targetPosition = 0.0;
// Initialize objects
m_Kraken = new TalonFX(Constants.TALON_CAN_ID);
m_DriverOutput = new DutyCycleOut(0); // Initialize with 0% output

// start with factory-default configs
var currentConfigs = new MotorOutputConfigs();

// Set direction of rotation
currentConfigs.Inverted = InvertedValue.Clockwise_Positive;

// Apply configuration settings to motor driver
m_Kraken.getConfigurator().apply(currentConfigs);

// Initialize position encoder
m_Kraken.setPosition(0);

var talonFXConfigs = new TalonFXConfiguration();

// set slot 0 gains
var slot0Configs = talonFXConfigs.Slot0;
slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
slot0Configs.kI = 0; // no output for integrated error
slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

// set Motion Magic settings
var motionMagicConfigs = talonFXConfigs.MotionMagic;
motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

m_Kraken.getConfigurator().apply(talonFXConfigs);

    public ElevatorSubsystem() {

        if(Constants.kEnableElevator) {

            if (RobotBase.isReal()) {
                isSim = false;
            } else {
                isSim = true;
            }

            if (Constants.kEnableDebugElevator) {

                Shuffleboard.getTab("Elevator")
                    .addDouble("Position", this::getPosition)
                    .withWidget(BuiltInWidgets.kTextView);

                Shuffleboard.getTab("Elevator")
                    .addDouble("Target", this::getPosition)
                    .withWidget(BuiltInWidgets.kTextView);

                SmartDashboard.putData(this);
                Shuffleboard.getTab("Elevator").add(this);
            }
        }
    }

    public void setDesiredState(ElevatorState state) {
        if(this.state == state) {
            // trying to set the state to the state we are already at
            // just returning to save cycles
            return;
        }

            switch (state) {
                case Start:
                    targetPosition = Constants.ElevatorConstants.Start;
                    break;
                case CoralHuman:
                    targetPosition = Constants.ElevatorConstants.CoralHuman;
                    break;
                case CoralL4:
                    targetPosition = Constants.ElevatorConstants.CoralL4;
                    break;
                case CoralL3:
                    targetPosition = Constants.ElevatorConstants.CoralL3;
                    break;
                case CoralL2:
                    targetPosition = Constants.ElevatorConstants.CoralL2;
                    break;
                case CoralL1:
                    targetPosition = Constants.ElevatorConstants.CoralL1;
                    break;
                case AlgaeL3:
                    targetPosition = Constants.ElevatorConstants.AlgaeL3;
                    break;
                case AlgaeL2:
                    targetPosition = Constants.ElevatorConstants.AlgaeL2;
                    break;
                case AlgaeL1:
                    targetPosition = Constants.ElevatorConstants.AlgaeL1;
                    break;
                case AlgaeShoot:
                    targetPosition = Constants.ElevatorConstants.AlgaeShoot;
                    break;
                case AlgaeHuman:
                    targetPosition = Constants.ElevatorConstants.AlgaeHuman;
                    break;
                case AlgaeFloor:
                    targetPosition = Constants.ElevatorConstants.AlgaeFloor;
                    break;
                default:
                    targetPosition = 0.0;
                    break;
            }
        //}

        this.state = state;
    }

    public ElevatorState getState() {
        return state;
    }

    @Override
	public void periodic() {

        if (Constants.kEnableElevator) {
           	// create a Motion Magic request, voltage output
		final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
		
		// set target position 
		m_Kraken.setControl(m_request.withPosition(targetPosition));
        }
    }

    private void setConfig() {
        
    }

    public double getPosition() {

        if(isSim) {
            return motorSim.getRelativeEncoderSim().getPosition();
        }

    }

    public double getTargetPosition() {

        return this.targetPosition;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }
public double getJerk() {
        return this.motionMagicConfigs.MotionMagicJerk;
    }
public double getCruise() {
        return this.motionMagicConfigs.MotionMagicCruiseVelocity;
    }
public double getP() {
        return this.slot0Configs.kP;
    }
public void setJerk(double jerk) {
        this.motionMagicConfigs.MotionMagicJerk = jerk;
    }
public void setCruise(double cruise) {
        this.motionMagicConfigs.MotionMagicCruiseVelocity = cruise;
    }
public void setP(double p) {
        this.slot0Configs.kP = p;
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        //builder.setActuator(true);
        //builder.setSafeState(this::disable);
        builder.addDoubleProperty("Jerk", this::getJerk, this::setJerk);
        builder.addDoubleProperty("Cruise", this::getCruise, this::setCruise);
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("Target", this::getTargetPosition, this::setTargetPosition);
        builder.addDoubleProperty("Position", this::getPosition,null);
    }
}
