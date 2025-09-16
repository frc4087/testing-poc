package vv.utils;

import java.util.Timer;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.concurrent.TimeUnit;

import org.awaitility.Awaitility;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import vv.config.VVConfig;

public class TestSetup {
    public static final VVConfig CONFIG = VVConfig.readFromPath("./src/test/resources/test.properties");
    public static final Double POSITION_TEST_TOLERANCE = 0.15;
    public static final Double VELOCITY_TEST_TOLERANCE_MPS = 0.15;
    public static final Double ROTATIONAL_TEST_TOLERANCE_DEG = 5.0;
    public static final Double ROTATIONAL_VELOCITY_TEST_TOLERANCE_RAD_PER_SEC = DegreesPerSecond.of(0.5).in(RadiansPerSecond);

    public static void resetSimulationState() {
        if (!HAL.initialize(500, 0)) {
            throw new RuntimeException("HAL initialization failed");
        }

        RobotController.resetRailFaultCounts();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
    
        DriverStationSim.resetData();
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
		DriverStation.silenceJoystickConnectionWarning(true);
        SimHooks.setProgramStarted();
        
        var simLoop = CONFIG.simulation().simLoopPeriodFreq().asPeriod();
        Awaitility.setDefaultPollInterval((long)simLoop.baseUnitMagnitude(), TimeUnit.SECONDS);
    }

    /**
     * You may be tempted to use {@link Command#isFinished()} instead of this method,
     * but this approach is more reliable in the context of simulation because it returns
     * true as soon as the scheduler is finished with the command, rather than on the command's
     * internal state. This resolves an problem I had in testing where the commands had all run
     * but the aggregating sequence command was not returning finished. 
     */
    public static boolean isFinished(Command cmd) {
        return !CommandScheduler.getInstance().isScheduled(cmd);
    }

    public static long maxSimulationTime(long distanceMeters) {
        var maxVel = CONFIG.pid().translationalConstraints().maxV();
        var maxAccel = CONFIG.pid().translationalConstraints().maxA();
        var expectedTime = calculateTrapezoidTime(distanceMeters, maxVel, maxAccel);
        return (long) Math.ceil(expectedTime * 2);
    }
    
    private static double calculateTrapezoidTime(long distance, double maxVel, double maxAccel) {
        var accelTime = maxVel / maxAccel;
        var accelDistance = 0.5 * maxAccel * accelTime * accelTime;
        
        if (2 * accelDistance >= distance) {
            return 2 * Math.sqrt(distance / maxAccel);
        } else {
            var cruiseDistance = distance - 2 * accelDistance;
            var cruiseTime = cruiseDistance / maxVel;
            return 2 * accelTime + cruiseTime;
        }
    }
}
