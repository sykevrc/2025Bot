package frc.robot.subsystems;


import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

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
        AlgaeFloor,
        ClimberUp,
        ClimberDown
    }

    private boolean isSim = false;
    private ElevatorState state = ElevatorState.Start;
    private double targetPosition = 0.0;
    //private SparkMax motor = null;
    private DutyCycleEncoder encoder = null;
    private TalonFX motor = null;
    //private SparkMaxSim motorSim = null;
    //private SparkMax motor2 = null;
    private TalonFX motor2 = null;
    //private SparkMaxSim motor2Sim = null;
    //private SparkClosedLoopController pid = null;

    // create a Motion Magic request, voltage output
  //  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    //private SparkMaxConfig config = new SparkMaxConfig();

    private double p = Constants.ElevatorConstants.P;
    private double i = Constants.ElevatorConstants.I;
    private double d = Constants.ElevatorConstants.D;

    private ProfiledPIDController profiledPIDController = new ProfiledPIDController(
        p,
        i, 
        d, 
        new TrapezoidProfile.Constraints(2, 2),
        0.02
    ); 

    
    // these values were calculate using https://www.reca.lc/linear

    private double currentPosition = 0.0;
    private double previousValue = 0.0;
    private int revolutionCount = 0;

    

    public ElevatorSubsystem() {

        if(Constants.kEnableElevator) {

            if (RobotBase.isReal()) {
                isSim = false; 
            } else {
                isSim = true;
            }
            
            encoder = new DutyCycleEncoder(1, 30, 16.484); 
            motor = new TalonFX(Constants.ElevatorConstants.motor_id, Constants.kCanivoreCANBusName);
            motor2 = new TalonFX(Constants.ElevatorConstants.motor2_id, Constants.kCanivoreCANBusName);
                
        

            setConfig();
  
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
                case ClimberUp:
                    targetPosition = Constants.ElevatorConstants.ClimberUp;
                    break;
                case ClimberDown:
                    targetPosition = Constants.ElevatorConstants.ClimberDown;
                    break;
                default:
                    targetPosition = 0.0;
                    break;
            }

        this.state = state;
    }

    public ElevatorState getState() {
        return state;
    }

    @Override
	public void periodic() {

        if (Constants.kEnableElevator) {
            
            currentPosition = encoder.get();
            // set target position to 100 rotations
            motor.set(
                profiledPIDController.calculate(currentPosition, targetPosition)
            );//(m_request.withPosition(targetPosition));
        }
    }

    private void setConfig() {

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        // var slot0Configs = talonFXConfigs.Slot0;
        // slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        // slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        // slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        // slot0Configs.kP = Constants.ElevatorConstants.P; // A position error of 2.5 rotations results in 12 V output
        // slot0Configs.kI = Constants.ElevatorConstants.I; // no output for integrated error
        // slot0Configs.kD = Constants.ElevatorConstants.D; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        // var motionMagicConfigs = talonFXConfigs.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
        // motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        // motionMagicConfigs.MotionMagicJerk = Constants.ElevatorConstants.MMJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

        motor.getConfigurator().apply(talonFXConfigs);

        // Set motor 2 to follow motor 1
        motor2.setControl(new Follower(Constants.ElevatorConstants.motor_id, false));
        
        encoder.setInverted(true);

    }

    public double getPosition() {

        /*if(isSim) {
            return motorSim.getRelativeEncoderSim().getPosition();
        }*/

        //return motor.getPosition().getValueAsDouble();
        return encoder.get();
        
    }

    public double getTargetPosition() {

        return this.targetPosition;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public boolean atTargetPosition() {
        // return true if we are just about at the target position
        return Math.abs(targetPosition - currentPosition) < 0.05;
    }

    public double getP() {
        return this.p;
    }

    public void setP(double p) {
        this.p = p;
        setConfig();
    }

    public double getI() {
        return this.i;
    }

    public void setI(double i) {
        this.i = i;
        setConfig();
    }

    public double getD() {
        return this.d;
    }

    public void setD(double d) {
        this.d = d;
        setConfig();
    }

    public int getRevolutions() {
        return this.revolutionCount;
    }

    public void resetEncoder() {
        //motor.getEncoder().setPosition(0.0);
        motor.setPosition(0.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        //builder.setActuator(true);
        //builder.setSafeState(this::disable);
        builder.addDoubleProperty("D", this::getD, this::setD);
        builder.addDoubleProperty("I", this::getI, this::setI);
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("Target", this::getTargetPosition, this::setTargetPosition);
        builder.addDoubleProperty("Position", this::getPosition,null);
        builder.addBooleanProperty("At Target Position", this::atTargetPosition,null);
    }
}
