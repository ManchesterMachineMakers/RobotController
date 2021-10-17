package org.firstinspires.ftc.teamcode.diagnostics.util;

public interface Selector<T> {
    <S extends T> void select(S inst);
    <S extends T> S get();
}
