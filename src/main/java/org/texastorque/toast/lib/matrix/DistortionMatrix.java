package org.texastorque.toast.lib.matrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N1;

public final class DistortionMatrix extends Matrix<N5, N1> {
    public DistortionMatrix(Matrix<N5, N1> other) {
        super(other);
    }
}
