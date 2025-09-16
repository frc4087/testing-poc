package vv.subsystems.roller;

import java.util.Optional;

import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import vv.config.VVConfig;
import vv.sims.TalonFXSim;

public final class RollerSubsystem extends SubsystemBase implements AutoCloseable {

    private final Timer cmdTimer = new Timer();
    private Optional<Double> duration = Optional.empty();
    private RollerState state = RollerState.IDLING;

    private final RollerMotor motor;
    private final RollerLED led;

    public enum RollerState {
        INTAKING,
        OUTPUTTING,
        IDLING
    }

    public RollerSubsystem(VVConfig config) {
        motor = new RollerMotor(config.roller().motor(), config.simulation());
        led = new RollerLED(config.roller().led());
    }

    @Override
    public void periodic() {
        if (duration.isPresent() && cmdTimer.get() > duration.get()) {
            setState(RollerState.IDLING, Optional.empty());
        }

        switch (state) {
            case IDLING -> {
                this.motor.idle();
                this.led.off();
            }
            case INTAKING -> {
                this.motor.intake();
                this.led.setColor(Color.kBlue);
            }
            case OUTPUTTING -> {
                this.motor.output();
                this.led.setColor(Color.kGreen);
            }
        }
    }

    @Override
    public void simulationPeriodic() { 
        motor.sim().ifPresent(TalonFXSim::updateState);
    }

    public RollerState getState() {
        return state;
    }

    public RollerMotor getMotor() {
        return motor;
    }

    public RollerLED getLED() {
        return led;
    }

    private void setState(RollerState state, Optional<Double> duration) {
        this.state = state;
        this.duration = duration;
        if (duration.isPresent()) {
            startTimer(duration.get());
        } else {
            resetTimer();
        }
    }

    public Command intake() {
        return this.runOnce(
            () -> setState(RollerState.INTAKING, Optional.empty())
        ).withName("RollerIntake");
    }

    public Command intake(Time duration) {
        return this.runOnce(
            () -> setState(RollerState.INTAKING, Optional.ofNullable(duration.in(Seconds)))
        ).withName("RollerIntakeTimed");
    }

    public Command output() {
        return this.runOnce(
            () -> setState(RollerState.OUTPUTTING, Optional.empty())
        ).withName("RollerOutput");
    }

    public Command output(Time duration) {
        return this.runOnce(
            () -> setState(RollerState.OUTPUTTING, Optional.ofNullable(duration.in(Seconds)))
        ).withName("RollerOutputTimed");
    }

    public Command idle() {
        return this.runOnce(
            () -> setState(RollerState.IDLING, Optional.empty())
        ).withName("RollerIdle");
    }

    private void resetTimer() {
        cmdTimer.stop();
        cmdTimer.reset();
        this.duration = Optional.empty();
    }

    private void startTimer(double duration) {
        cmdTimer.reset();
        this.duration = Optional.of(duration);
        cmdTimer.start();
    }

    @Override
    public void close() throws Exception {
        led.close();
        motor.idle();
    }
}
