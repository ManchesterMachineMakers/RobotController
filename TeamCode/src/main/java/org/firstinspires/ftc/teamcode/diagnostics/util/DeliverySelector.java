package org.firstinspires.ftc.teamcode.diagnostics.util;

import org.firstinspires.ftc.teamcode.subassemblies.Delivery;

public class DeliverySelector implements Selector<Delivery> {
    Delivery delivery;
    public DeliverySelector(Delivery delivery) {
        select(delivery);
    }
    public <S extends Delivery> void select(S inst) {
        this.delivery = inst;
    }

    public <S extends Delivery> S get() {
        return (S) this.delivery;
    }
}
