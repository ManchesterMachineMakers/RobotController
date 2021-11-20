package org.firstinspires.ftc.teamcode.diagnostics.util;

public interface Selector<T> {
    Class<T> type();
    <S extends T> void select(S inst);
    <S extends T> S get();
    class NoneFound<T> implements Selector<T> {
        private Class<T> _type;
        public NoneFound(Class<T> type) {
            this._type = type;
        }
        public Class<T> type() {
            return this._type;
        }

        public <S extends T> void select(S inst) {

        }

        public <S extends T> S get() {
            return (S) null;
        }
    }
    static <T> Selector<T> get(Selector<?>[] selectors, Class<T> type) {
        for (Selector selector : selectors) {
            if(selector.type() == type) return selector;
        }
        return new Selector.NoneFound<T>(type);
    }
    static <T> Selector<T> getOrDie(Selector<?>[] selectors, Class<T> type) throws Exception {
        Selector<T> sel = get(selectors, type);
        if(sel.getClass() == NoneFound.class) {
            throw new Exception("Selector for " + type.getName() + " not found, aborting.");
        }
        return sel;
    }
}
