/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.util.TorqueUtil;

public final class Lights extends TorqueSubsystem implements Subsystems {
    private static volatile Lights instance;

    private final AddressableLED leds;
    private final AddressableLEDBuffer buff;

    private static final int LENGTH = 20;

    private Lights() {
        leds = new AddressableLED(0);
        leds.setLength(LENGTH);
        buff = new AddressableLEDBuffer(LENGTH);
    }

    public static final Color getAllianceColor() {
        return DriverStation.getAlliance() == Alliance.Blue ? Color.kBlue
                                                            : Color.kRed;
    }

    public static final Color getAllianceColorFIRST() {
        return DriverStation.getAlliance() == Alliance.Blue ? Color.kFirstBlue
                                                            : Color.kFirstRed;
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        leds.start();
    }

    private LightAction solidGreen = new Solid(() -> Color.kGreen),
                        solidAlliance =
                            new Solid(() -> getAllianceColorFIRST()),
                        blinkGreen = new Blink(() -> Color.kGreen, 3),
                        blinkAlliance =
                            new Blink(() -> getAllianceColorFIRST(), 3),
                        solidPurple = new Solid(() -> Color.kPurple),
                        solidYellow = new Solid(() -> Color.kYellow),
                        blinkPurple = new Blink(() -> Color.kPurple, 3),
                        blinkYellow = new Blink(() -> Color.kYellow, 3),
                        solidRainbow = new Rainbow();

    public final LightAction getColor() {

        if (indexer.isIntaking() || arm.isAtShelf()) {
            if (indexer.getLastWantedGamePiece() == GamePiece.CUBE)
                return solidPurple;
            if (indexer.getLastWantedGamePiece() == GamePiece.CONE)
                return solidYellow;
        }

        if (drivebase.isState(Drivebase.State.ALIGN)) {
            if (drivebase.isPathAlignDone())
                return blinkGreen;
            return solidGreen;
        }

        if (drivebase.isState(Drivebase.State.BALANCE)) {
            if (drivebase.isAutoLevelDone())
                return blinkGreen;
            return solidGreen;
        }

        return solidAlliance;
    }

    @Override
    public final void update(final TorqueMode mode) {
        getColor().run(buff);
        leds.setData(buff);
    }

    private static abstract class LightAction {
        public abstract void run(AddressableLEDBuffer buff);
    }

    public static class Solid extends LightAction {
        private final Supplier<Color> color;

        public Solid(final Supplier<Color> color) { this.color = color; }

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

        public Blink(final Supplier<Color> color1, final Supplier<Color> color2,
                     final double hertz) {
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
                final int hue =
                    (rainbowFirstPixelHue + (i * 180 / buff.getLength())) % 180;
                buff.setHSV(i, hue, 255, 128);
            }
            rainbowFirstPixelHue += 3;
            rainbowFirstPixelHue %= 180;
        }
    }

    public static final synchronized Lights getInstance() {
        return instance == null ? instance = new Lights() : instance;
    }
}
