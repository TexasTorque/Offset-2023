package org.texastorque.controllers;

import java.util.Optional;

import org.texastorque.controllers.SwerveAlignController.AlignState;
import org.texastorque.controllers.SwerveAlignController.GridState;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface IAlignmentController {
    public void resetIf(final boolean notInLoop);
    public Optional<ChassisSpeeds> calculateAlignment();

    public void setAlignment(final AlignState alignment);
    public void setGridOverride(final GridState gridOverride);

    public boolean needsFieldRelative();

}
