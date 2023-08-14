package org.texastorque.toast.lib.matrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;

public final class IntrinsicsMatrix extends Matrix<N3, N3> {
    public IntrinsicsMatrix(Matrix<N3, N3> other) {
        super(other);
    }
}
