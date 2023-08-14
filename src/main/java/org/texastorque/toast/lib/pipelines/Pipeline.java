package org.texastorque.toast.lib.pipelines;

import java.util.function.Consumer;

import org.texastorque.toast.lib.matrix.DistortionMatrix;
import org.texastorque.toast.lib.matrix.IntrinsicsMatrix;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class Pipeline {
    protected static final ObjectMapper mapper = new ObjectMapper();
    protected static final void parseAndHandleJSON(final String json, final Consumer<JsonNode> handler) {
        try {
            handler.accept(mapper.readTree(json));
        } catch (final Exception e) {
            System.out.println(e.getMessage());
        }
    }
    private static final double[] convertNumberArrayToDoubleArray(final Number[] numbers) {
        final double[] doubles = new double[numbers.length];
        for (int i = 0; i < numbers.length; i++) doubles[i] = numbers[i].doubleValue();
        return doubles;
    }
    private static final double[] readNumberArrayToDoubleArray(final NetworkTableEntry ntEntry) {
        return convertNumberArrayToDoubleArray(ntEntry.getNumberArray(new Number [] {}));
    }

    public String name;

    public final String type;

    protected NetworkTableEntry dataEntry, typeEntry, intrEntry, distEntry;
    public Pipeline(final String type) {
        this.type = type;
    }

    public final void setName(final String name) {
        this.name = name;
        final NetworkTable toastTable = NetworkTableInstance.getDefault().getTable("toast");
        this.dataEntry = toastTable.getEntry(name + "_data");
        this.typeEntry = toastTable.getEntry(name + "_type");
        this.intrEntry = toastTable.getEntry(name + "_intr");
        this.distEntry = toastTable.getEntry(name + "_dist");
    }
    public final void runInit() {
        typeEntry.setString(type);
        init();
    }

    public final void runDeinit() {
        typeEntry.setString("none");
        deinit();
    }
    public final void runUpdate() { 
        final String json = dataEntry.getString("{}");
        parseAndHandleJSON(json, root -> update(root));
    }

    protected abstract void init();

    protected abstract void deinit();

    protected abstract void update(final JsonNode root);

    protected IntrinsicsMatrix getIntrinsics() {
        final double[] vals = readNumberArrayToDoubleArray(intrEntry);
        if (vals.length != 9) return null;
        return new IntrinsicsMatrix(new MatBuilder<>(Nat.N3(), Nat.N3()).fill(vals));
    }

    protected DistortionMatrix getDistorion() {
        final double[] vals = readNumberArrayToDoubleArray(distEntry);
        if (vals.length != 5) return null;
        return new DistortionMatrix(new MatBuilder<>(Nat.N5(), Nat.N1()).fill(vals));
    }

/*
 * local2_type | "april-tags"                   (string)
 * local2_data | "{}"                           (user defined json)
 * local2_intr | [1, 2, 3, 4, 5, 6, 7, 8, 9]    (double array, length 9, flattened 3x3 matrix)
 * local2_dist | [1, 2, 3, 4, 5]                (double array, length 5)
 */
}
