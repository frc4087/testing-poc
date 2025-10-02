package vv.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import vv.config.ArmConfig;
import vv.config.VVConfig;
import vv.sims.TalonFXSim;

public class ArmSubsystem extends SubsystemBase {
    private final ArmMotor motor;
    private final ArmConfig config;
    private ArmState state = ArmState.RESET;

    public enum ArmState {
        SNAP_UP,
        SNAP_DOWN,
        RUNNING_UP,
        RESET
    }

    public ArmSubsystem(VVConfig config) {
        this.config = config.arm();
        motor = new ArmMotor(config.arm().motor(), config.simulation());
    }

    @Override
    public void periodic() {
        switch (state) {
            case SNAP_UP -> {
                this.motor.setControlPosition(config.upSetpoint());
            }
            case SNAP_DOWN -> {
                this.motor.setControlPosition(config.downSetpoint());
            }
            case RUNNING_UP -> {
                this.motor.setSpeed(config.resetSpeed());
            }
            case RESET -> {
                this.motor.reset();
            }
        }
    }
    @Override
    public void simulationPeriodic() { 
        motor.sim().ifPresent(TalonFXSim::updateState);
    }

    public ArmState getState() {
        return state;
    }

    public ArmMotor getMotor() {
        return motor;
    }

    public void setState(ArmState state) {
        this.state = state;
    }

	public Command snapUp() {
		return this.run(() -> setState(ArmState.SNAP_UP))
            .withName("Arm::SnapUp");
	}

	public Command snapDown() {
		return this.run(() -> setState(ArmState.SNAP_DOWN))
            .withName("Arm::SnapDown");
	}

	public Command runUp() {
		return this.runEnd(
                () -> setState(ArmState.RUNNING_UP),
                () -> setState(ArmState.RESET)
            )
            .withName("Arm::RunUp");
	}
}
