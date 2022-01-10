package org.firstinspires.ftc.teamcode.diagnostics.util;

public interface Testable {
    static <T extends Testable> T get(Testable[] selectors, Class<T> type) {
        for (Testable selector : selectors) {
            if(selector.getClass() == type) return (T) selector;
        }
        return null;
    }
    static <T extends Testable> T getOrDie(Testable[] selectors, Class<T> type) throws Exception {
        T sel = get(selectors, type);
        if(sel == null) {
            throw new Exception("Selector for " + type.getName() + " not found, aborting.");
        }
        return sel;
    }
}
