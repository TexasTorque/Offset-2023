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

        public Solid(final Supplier<Color> color) {
            this.color = color;
        }

        @Override
        public void run(AddressableLEDBuffer buff) {
            for (int i = 0; i < buff.getLength(); i++)
                buff.setLED(i, color.get());
        }
    }

    public static class Blink extends LightAction {
        private final Supplier<Color> color1, color2;
        private final double hertz;

        public Blink(final Supplier<Color> color1, final double hertz) {
            this(color1, () -> Color.kBlack, hertz);
        }

        public Blink(final Supplier<Color> color1, final Supplier<Color> color2, final double hertz) {
            this.color1 = color1;
            this.color2 = color2;
            this.hertz = hertz;
        }

        @Override
        public void run(AddressableLEDBuffer buff) {
            final double timestamp = TorqueUtil.time();
            final boolean on = (Math.floor(timestamp * hertz) % 2 == 1);
            for (int i = 0; i < buff.getLength(); i++)
                buff.setLED(i, on ? color1.get() : color2.get());
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

    private static abstract class LightAction {
        public abstract void run(AddressableLEDBuffer buff);
    }

    private static volatile Lights instance;

    private static final int LENGTH = 50;

    public static final Color getAllianceColorLight() {
        return DriverStation.getAlliance() == Alliance.Blue ? Color.kLightBlue : Color.kPink;
    }

    public static final Color getAllianceColor() {
        return DriverStation.getAlliance() == Alliance.Blue ? Color.kBlue : Color.kRed;
    }

    public static final Color getAllianceColorFIRST() {
        return DriverStation.getAlliance() == Alliance.Blue ? Color.kFirstBlue : Color.kFirstRed;
    }

    public static final synchronized Lights getInstance() {
        return instance == null ? instance = new Lights() : instance;
    }

    private final AddressableLED superstructureLEDs;

    private final AddressableLEDBuffer buff;

    private LightAction 
            solidGreen = new Solid(() -> Color.kGreen), solidAlliance = new Solid(() -> getAllianceColor()),
            blinkLightAlliance = new Blink(() -> getAllianceColorLight(), 6),
            blinkGreen = new Blink(() -> Color.kGreen, 6),
            solidPurple = new Solid(() -> Color.kPurple), solidYellow = new Solid(() -> Color.kYellow),
            blinkPurple = new Blink(() -> Color.kPurple, 6), blinkYellow = new Blink(() -> Color.kYellow, 6),
            solidRed = new Solid(() -> Color.kRed),
            rainbow = new Rainbow(), blinkRed = new Blink(() -> Color.kRed, 6);

    private Lights() {
        superstructureLEDs = new AddressableLED(Ports.LIGHTS_SUPERSTRUCTURE);
        superstructureLEDs.setLength(LENGTH);

        buff = new AddressableLEDBuffer(LENGTH);

        for (int i = 0; i < buff.getLength(); i++)
            buff.setLED(i, Color.kGreen);

        superstructureLEDs.setData(buff);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        superstructureLEDs.start();

        mode.onDisabled(() -> {
            for (int i = 0; i < buff.getLength(); i++)
                buff.setLED(i, getAllianceColor());
            superstructureLEDs.setData(buff);
        });
    }

    public final LightAction getColor(final TorqueMode mode) {
        if (drivebase.isState(Drivebase.State.ALIGN)) {
            if (drivebase.isPathAlignDone())
                return blinkGreen;
            return solidGreen;
        }

        if (forks.isForksRunning()) {
            return rainbow;
        }

        if (drivebase.isState(Drivebase.State.BALANCE)) {
            if (drivebase.isAutoLevelDone()) {
                if (mode.isAuto())
                    return rainbow;
                return blinkLightAlliance;
            }
            return solidGreen;
        }

        final boolean blinkColor = spindexer.isAutoSpindexing() || intake.isIntaking() || arm.isDoingHandoff()
                || drivebase.getSpeedSetting().isSlow();

        if (hand.isCubeMode()) {
            return blinkColor ? blinkPurple : solidPurple;
        } else if (hand.isConeMode()) {
            return blinkColor ? blinkYellow : solidYellow;
        }

        return solidAlliance;
    }

    @Override
    public final void update(final TorqueMode mode) {
        getColor(mode).run(buff);
        superstructureLEDs.setData(buff);
    }
}