package vv.subsystems.roller;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import vv.config.RollerConfig;

public class RollerLED implements AutoCloseable {
    
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private Optional<Color> color = Optional.empty();

    public RollerLED(RollerConfig.LEDConfig config) {
        led = new AddressableLED(config.port());
        buffer = new AddressableLEDBuffer(120);
        led.setLength(buffer.getLength());
    }

    public void setColor(Color color) {
        setPattern(LEDPattern.solid(color));
        this.color = Optional.ofNullable(color);
    }

    public void off() {
        setPattern(LEDPattern.kOff);
        this.color = Optional.empty();
    }

    public Optional<Color> getColor() {
        return this.color;
    }

    private void setPattern(LEDPattern pattern) {
        pattern.applyTo(buffer);
        led.setData(buffer);
    }


    @Override
    public void close() throws Exception {
        led.close();
    }

}
