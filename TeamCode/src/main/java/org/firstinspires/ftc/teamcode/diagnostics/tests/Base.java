package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.teamcode.diagnostics.Runner;
import org.firstinspires.ftc.teamcode.diagnostics.util.Selectors;

public interface Base {
    boolean run(Selectors sel, Runner runner) throws InterruptedException;
}
