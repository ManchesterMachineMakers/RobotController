package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;

public interface Base {
    Class<? extends Testable>[] requires();
    boolean run(Testable[] sel, Runner runner) throws Exception;
}
