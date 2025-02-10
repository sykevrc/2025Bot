package frc.robot.subsystems;


import java.util.EnumSet;
import java.util.Map;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffectorSubsystem extends SubsystemBase {
    public enum EndEffectorState {
        Stopped,
        IntakeAlgaeFloor,
        IntakeCoralHumanElement,
        EjectAlgaeFloor,
        EjectCoral,
    }

    private boolean isSim = false;
    private EndEffectorState state = EndEffectorState.Stopped;
    private double targetVelocity1 = 0.0;
    private double targetVelocity2 = 0.0;
    private SparkFlex motor = null;
    private SparkFlexSim motorSim = null;
    private SparkClosedLoopController pid = null;
    private SparkFlexConfig config = new SparkFlexConfig();
    private double p = Constants.EndEffectorConstants.P;
    private double i = Constants.EndEffectorConstants.I;
    private double d = Constants.EndEffectorConstants.D;

    private SparkMax motor2 = null;
    private SparkMaxSim motor2Sim = null;
    private SparkClosedLoopController pid2 = null;
    private SparkMaxConfig config2 = new SparkMaxConfig();

    private boolean hasItem = false;

    private DigitalInput beamBreaker = new DigitalInput(Constants.EndEffectorConstants.kBeamBreakerPort);

    public EndEffectorSubsystem() {
        if(Constants.kEnableEndEffector) {
            if (RobotBase.isReal()) {
                isSim = false;
            } else {
                isSim = true;
            }

            motor = new SparkFlex(Constants.EndEffectorConstants.motor_id, MotorType.kBrushless);
            motor2 = new SparkMax(Constants.EndEffectorConstants.motor2_id, MotorType.kBrushless);

            if(isSim) {
			    motorSim = new SparkFlexSim(motor, DCMotor.getNeoVortex(1));
                motor2Sim = new SparkMaxSim(motor2, DCMotor.getNEO(1));
		    }

            setConfig();

		    pid = motor.getClosedLoopController();
            pid2 = motor2.getClosedLoopController();

            if (Constants.kEnableDebugEndEffector) {

                Shuffleboard.getTab("End Effector")
                    .addDouble("Velocity1", this::getVelocity1)
                    .withWidget(BuiltInWidgets.kTextView);

                    Shuffleboard.getTab("End Effector")
                    .addDouble("Velocity2", this::getVelocity1)
                    .withWidget(BuiltInWidgets.kTextView);

                Shuffleboard.getTab("End Effector")
                    .addDouble("Target Velocity 1", this::getTargetVelocity1)
                    .withWidget(BuiltInWidgets.kTextView);

                Shuffleboard.getTab("End Effector")
                    .addDouble("Target Velocity 2", this::getTargetVelocity2)
                    .withWidget(BuiltInWidgets.kTextView);

                SmartDashboard.putData(this);
                Shuffleboard.getTab("End Effector").add(this);

            }   
        }
    }

    public void setDesiredState(EndEffectorState state) {

        // Are we sending the same state again?  If so act like a toggle and stop
        if (this.state == state) {
            targetVelocity1 = Constants.EndEffectorConstants.StoppedMotor1;
            targetVelocity2 = Constants.EndEffectorConstants.StoppedMotor2;
        } else {

            switch (state) {
                case Stopped:
                    targetVelocity1 = Constants.EndEffectorConstants.StoppedMotor1;
                    targetVelocity2 = Constants.EndEffectorConstants.StoppedMotor2;
                    break;
                case IntakeAlgaeFloor:
                    targetVelocity1 = Constants.EndEffectorConstants.IntakeAlgaeFloorMotor1;
                    targetVelocity2 = Constants.EndEffectorConstants.IntakeAlgaeFloorMmotor2;
                    hasItem = false;
                    break;
                case IntakeCoralHumanElement:
                    targetVelocity1 = Constants.EndEffectorConstants.IntakeCoralHumanElementMotor1;
                    targetVelocity2 = Constants.EndEffectorConstants.IntakeCoralHumanElementMotor2;
                    hasItem = false;
                    break;
                case EjectAlgaeFloor:
                    targetVelocity1 = Constants.EndEffectorConstants.EjectAlgaeFloorMotor1;
                    targetVelocity2 = Constants.EndEffectorConstants.EjectAlgaeFloorMotor2;
                    hasItem = false;
                    break;
                case EjectCoral:
                    targetVelocity1 = Constants.EndEffectorConstants.EjectCoralMotor1;
                    targetVelocity2 = Constants.EndEffectorConstants.EjectCoralMotor2;
                    hasItem = false;
                    break;
                default:
                    targetVelocity1 = 0.0;
                    targetVelocity2 = 0.0;
                    break;
            }
        }

        this.state = state;

        // drivePID.setReference(targetPosition, ControlType.kPosition);
    }

    @Override
	public void periodic() {

        if (Constants.kEnableEndEffector) {

            if(beamBreaker.get()) {
                this.hasItem = true;
            } else {
                this.hasItem = false;
            }

            // Do we have the item?
            if(
                motor.getOutputCurrent() >= Constants.EndEffectorConstants.OutputCurrentLimitMotor1
                || motor2.getOutputCurrent() >= Constants.EndEffectorConstants.OutputCurrentLimitMotor2
            ) {
                // Looks like we got the item, so stop the end effector motors
                state = EndEffectorState.Stopped;
                hasItem = true;
            }

            if (!hasItem) {
                // We don't have it so run

                pid.setReference(targetVelocity1, ControlType.kVelocity);
                pid2.setReference(targetVelocity2, ControlType.kVelocity);

                if (isSim) {
                    motorSim.iterate(
                            // 0.1,
                            // desiredState.speedMetersPerSecond,
                            motor.getOutputCurrent(),
                            RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                            0.02);

                    motor2Sim.iterate(
                            motor2.getOutputCurrent(),
                            RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                            0.02);
                }
            } else {
                // we have it so stop the wheels
                targetVelocity1 = 0.0;
                targetVelocity2 = 0.0;
                pid.setReference(targetVelocity1, ControlType.kVelocity);
                pid2.setReference(targetVelocity2, ControlType.kVelocity);
            }
        }
    }

    private void setConfig() {
        // Vortex
        config = new SparkFlexConfig();

        config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
		    .pid(
			    p, 
                i, 
                d
			);
        config.signals.primaryEncoderPositionPeriodMs(5);

        motor.configure(
		    config, 
			ResetMode.kResetSafeParameters, 
			PersistMode.kPersistParameters
		);

        // Neo 550
        config2 = new SparkMaxConfig();

        config2.inverted(true)
            .idleMode(IdleMode.kBrake);
        config2.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);

        config2.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
			    p, 
                i, 
                d
			);

        config2.signals.primaryEncoderPositionPeriodMs(5);
    }

    public boolean hasItem() {
        return this.hasItem;
    }

    public double getVelocity1() {

        if(isSim) {
            return motorSim.getRelativeEncoderSim().getVelocity();
        }

        return motor.getEncoder().getVelocity();
    }

    public double getVelocity2() {

        if(isSim) {
            return motor2Sim.getRelativeEncoderSim().getVelocity();
        }

        return motor2.getEncoder().getVelocity();
    }

    public double getTargetVelocity1() {

        return this.targetVelocity1;
    }

    public void setTargetVelocity1(double targetVelocity) {
        this.targetVelocity1 = targetVelocity;
    }

    public double getTargetVelocity2() {

        return this.targetVelocity1;
    }

    public void setTargetVelocity2(double targetVelocity) {
        this.targetVelocity2 = targetVelocity;
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty("D", this::getD, this::setD);
        builder.addDoubleProperty("I", this::getI, this::setI);
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("Target Velocity1", this::getTargetVelocity1, this::setTargetVelocity1);
        builder.addDoubleProperty("Target Velocity2", this::getTargetVelocity2, this::setTargetVelocity2);
        builder.addDoubleProperty("Velocity1", this::getVelocity1, null);
        builder.addDoubleProperty("Velocity2", this::getVelocity2, null);
        builder.addBooleanProperty("Has Item", this::hasItem, null);
    }
}
