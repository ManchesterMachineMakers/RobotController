package org.firstinspires.ftc.teamcode.diagnostics.util;

import org.firstinspires.ftc.teamcode.util.Subassembly;

public interface Testable extends Subassembly {
    static <T extends Testable> T get(Testable[] selectors, Class<T> type) {
        for (Testable selector : selectors) {
            try {
                return type.cast(selector);
            } catch (ClassCastException ignored) {}
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
