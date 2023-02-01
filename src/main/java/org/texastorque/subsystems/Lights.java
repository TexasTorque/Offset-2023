package org.texastorque.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.util.TorqueUtil;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public final class Lights extends TorqueSubsystem implements Subsystems {
    private static volatile Lights instance;

    private final AddressableLED leds;
    private final AddressableLEDBuffer buff;

    private static final int LENGTH = 20;

    private List<LightAction> actions = new ArrayList<>();

    private Lights() {
        leds = new AddressableLED(0);
        leds.setLength(LENGTH);
        buff = new AddressableLEDBuffer(LENGTH);

        actions.add(new Solid(() -> ((indexer.isIntaking() || arm.isAtShelf()) && indexer.getLastWantedGamePiece() == GamePiece.CUBE), () -> Color.kPurple));
        actions.add(new Solid(() -> ((indexer.isIntaking() || arm.isAtShelf()) && indexer.getLastWantedGamePiece() == GamePiece.CONE), () -> Color.kYellow));
        actions.add(new Solid(() -> drivebase.isState(Drivebase.State.ALIGN), () -> Color.kGreen));
        actions.add(new Blink(() -> drivebase.isState(Drivebase.State.BALANCE), () -> Color.kGreen, 3)); 
        actions.add(new Solid(() -> drivebase.isState(Drivebase.State.FIELD_RELATIVE), () -> getAllianceColorFIRST()));
    }

    public static final Color getAllianceColor() {
        return DriverStation.getAlliance() == Alliance.Blue ? Color.kBlue : Color.kRed;
    }

    public static final Color getAllianceColorFIRST() {
        return DriverStation.getAlliance() == Alliance.Blue ? Color.kFirstBlue : Color.kFirstRed;
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        leds.start();
    }

    @Override
    public final void update(final TorqueMode mode) {
        for (final LightAction action : actions) {
            if (action.condition.getAsBoolean()) {
                action.run(buff);
                leds.setData(buff);
                break;
            }
        }
    }

    private static abstract class LightAction {
        public final BooleanSupplier condition; 

        public LightAction(final BooleanSupplier condition) {
            this.condition = condition;
        }

        public abstract void run(AddressableLEDBuffer buff);
    }

    public static class Solid extends LightAction {
        private final Supplier<Color> color;

        public Solid(final BooleanSupplier condition, final Supplier<Color> color) {
            super(condition);
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
    
        public Blink(final BooleanSupplier condition, final Supplier<Color> color1, final double hertz) {
            this(condition, color1, () -> Color.kBlack, hertz);
        }

        public Blink(final BooleanSupplier condition, final Supplier<Color> color1, final Supplier<Color> color2, final double hertz) {
            super(condition);
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

        public Rainbow(final BooleanSupplier condition) {
            super(condition);
        }

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

   

    public static final synchronized Lights getInstance() {
        return instance == null ? instance = new Lights() : instance;
    }
}
