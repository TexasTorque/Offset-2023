/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import java.util.function.Supplier;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.util.TorqueUtil;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public final class Lights extends TorqueSubsystem implements Subsystems {
    public static class Solid extends LightAction {
        private final Supplier<Color> color;

        public Solid(final Supplier<Color> color) { this.color = color; }

        @Override
        public void run(AddressableLEDBuffer buff) {
            for (int i = 0; i < buff.getLength(); i++) buff.setLED(i, color.get());
        }
    }

    public static class Blink extends LightAction {
        private final Supplier<Color> color1, color2;
        private final double hertz;

        public Blink(final Supplier<Color> color1, final double hertz) { this(color1, () -> Color.kBlack, hertz); }

        public Blink(final Supplier<Color> color1, final Supplier<Color> color2, final double hertz) {
            this.color1 = color1;
            this.color2 = color2;
            this.hertz = hertz;
        }

        @Override
        public void run(AddressableLEDBuffer buff) {
            final double timestamp = TorqueUtil.time();
            final boolean on = (Math.floor(timestamp * hertz) % 2 == 1);
            for (int i = 0; i < buff.getLength(); i++) buff.setLED(i, on ? color1.get() : color2.get());
        }
    }
    public static class Rainbow extends LightAction {
        private int rainbowFirstPixelHue = 0;

        @Override
        public void run(AddressableLEDBuffer buff) {
            for (var i = 0; i < buff.getLength(); i++) {
                final int hue = (rainbowFirstPixelHue + (i * 180 / buff.getLength())) % 180;
                buff.setHSV(i, hue, 255, 128);
            }
            rainbowFirstPixelHue += 3;
            rainbowFirstPixelHue %= 180;
        }
    }

    private static abstract class LightAction { public abstract void run(AddressableLEDBuffer buff); }

    private static volatile Lights instance;

    private static final int LENGTH = 20;

    public static final Color getAllianceColor() { return DriverStation.getAlliance() == Alliance.Blue ? Color.kBlue : Color.kRed; }

    public static final Color getAllianceColorFIRST() { return DriverStation.getAlliance() == Alliance.Blue ? Color.kFirstBlue : Color.kFirstRed; }

    public static final synchronized Lights getInstance() { return instance == null ? instance = new Lights() : instance; }

    private final AddressableLED leds;

    private final AddressableLEDBuffer buff;

    private LightAction solidGreen = new Solid(() -> Color.kGreen), solidAlliance = new Solid(() -> getAllianceColor()),
                        blinkGreen = new Blink(() -> Color.kGreen, 6), blinkAlliance = new Blink(() -> getAllianceColor(), 6),
                        solidPurple = new Solid(() -> Color.kPurple), solidYellow = new Solid(() -> Color.kYellow),
                        blinkPurple = new Blink(() -> Color.kPurple, 6), blinkYellow = new Blink(() -> Color.kYellow, 6), rainbow = new Rainbow();

    private Lights() {
        leds = new AddressableLED(Ports.LIGHTS);
        leds.setLength(LENGTH);
        buff = new AddressableLEDBuffer(LENGTH);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        leds.start();
    }

    public final LightAction getColor(final TorqueMode mode) {
        if (drivebase.isState(Drivebase.State.ALIGN)) {
            if (drivebase.isPathAlignDone()) return blinkGreen;
            return solidGreen;
        }

        if (drivebase.isState(Drivebase.State.BALANCE)) {
            if (drivebase.isAutoLevelDone()) {
                if (mode.isAuto()) return rainbow;
                return blinkGreen;
            }
            return solidGreen;
        }

        if (hand.isCubeMode()) return solidPurple;
        else if (hand.isConeMode()) return solidYellow;

        return solidAlliance;
    }

    @Override
    public final void update(final TorqueMode mode) {
        getColor(mode).run(buff);
        leds.setData(buff);
    }
}
