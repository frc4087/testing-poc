package vv.config;

import java.util.Objects;
import java.util.Properties;

import static vv.config.PropertyReaders.readDoubleProperty;
import static vv.config.PropertyReaders.readIntegerProperty;

public record RollerConfig(
    MotorConfig motor,
    LEDConfig led
) {
 
    public RollerConfig {
        Objects.requireNonNull(motor, "Motor config cannot be null");
        Objects.requireNonNull(led, "LED config cannot be null");
    }

    public static RollerConfig fromProperties(Properties props) {
        return new RollerConfig(
            MotorConfig.fromProperties(props),
            LEDConfig.fromProperties(props)
        );
    }

    public static record MotorConfig(
        Integer port,
        Double intakeSpeed,
        Double outputSpeed,
        Double idleSpeed
    ) {
        public MotorConfig {
            Objects.requireNonNull(port, "Port cannot be null");
            Objects.requireNonNull(intakeSpeed, "Intake speed cannot be null");
            Objects.requireNonNull(outputSpeed, "Output speed cannot be null");
            Objects.requireNonNull(idleSpeed, "Idle speed cannot be null");
        }

        public static MotorConfig fromProperties(Properties props) {
            return new MotorConfig(
                readIntegerProperty(props, "roller.motor.port"),
                readDoubleProperty(props, "roller.motor.intake.speed"),
                readDoubleProperty(props, "roller.motor.output.speed"),
                readDoubleProperty(props, "roller.motor.idle.speed")
            );
        }
    }

    public static record LEDConfig(
        Integer port
    ) {
        public LEDConfig {
            Objects.requireNonNull(port, "Port cannot be null");
            assert port >= 0 && port <= 9 : "LED Port must be a PWM port between 0 and 9";
        }

        public static LEDConfig fromProperties(Properties props) {
            return new LEDConfig(
                readIntegerProperty(props, "roller.led.port")
            );
        }
    }
}
