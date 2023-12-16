package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subassemblies.DriveBase;
import org.firstinspires.ftc.teamcode.subassemblies.Vision;
import org.manchestermachinemakers.easyop.Inject;
import org.manchestermachinemakers.easyop.Linear;
import org.manchestermachinemakers.easyop.Subassembly;
import org.manchestermachinemakers.easyop.util.logging.LogPriority;
import org.manchestermachinemakers.easyop.util.logging.Logger;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Consumer;
import java.util.stream.Collectors;

// STAGE I - DETECT CUSTOM ELEMENT //

class DetectElementState extends AutonomousState {
    public static int POS_LEFT = 0;
    public static int POS_CENTER = 0;
    public static int POS_RIGHT = 0;

    DetectElementState(CenterStageAutonomous opMode) {
        super(opMode);
    }

    /**
     * Finds the position of a recognition
     * @param r the recognition
     * @return the position (0 = left, 1 = center, 2 = right)
     */
    private int getRecognitionPosition(Recognition r) {
        var center = ((r.getLeft() + r.getRight()) / 2) / r.getImageWidth();

        if(center < 0.3) return POS_LEFT;
        if(0.3 < center && center < 0.6) return POS_CENTER;
        return POS_RIGHT;
    }

    @Override
    public Action loop() {
        var vision = use(Vision.class);

        var recognition = vision.getTfod().getRecognitions()
                .stream()
                .filter(r ->
                        r.getLabel().equals("DUCK") &&
                        r.getConfidence() > 0.9)
                .max((r1, r2) -> Float.compare(r1.getConfidence(), r2.getConfidence()));

        if(recognition.isPresent()) {
            return setState(PurplePixelState.class, getRecognitionPosition(recognition.get()));
        }

        return next();
    }
}

// STAGE II - MOVE & PLACE PURPLE PIXEL //

class PurplePixelState extends AutonomousState {
    int elementPosition;

    public PurplePixelState(CenterStageAutonomous opMode, int elementPosition) {
        super(opMode);
        this.elementPosition = elementPosition;
    }

    @Override
    Action loop() {
        var driveBase = use(DriveBase.class);



        return next();
    }
}

// STAGE III - MOVE TO BACKDROP & PLACE YELLOW PIXEL //

// STAGE IV - PARK //

// ENTRYPOINT & INTERNAL CODE //

interface Action extends Consumer<CenterStageAutonomous> {
    default boolean shouldStop() { return false; }
}

abstract class AutonomousState implements Logger {
    private CenterStageAutonomous opMode;
    AutonomousState(CenterStageAutonomous opMode) {
        this.opMode = opMode;
    }

    @SuppressWarnings("unchecked")
    <T extends Subassembly> T use(Class<T> cSub) {
        try {
            return (T) Arrays.stream(opMode.getClass().getDeclaredFields())
                    .filter(field -> Arrays.stream(field.getDeclaredAnnotations()).anyMatch(annotation -> Inject.class.isAssignableFrom(annotation.annotationType())))
                    .filter(field -> cSub.isAssignableFrom(field.getType()))
                    .findFirst()
                    .get()
                    .get(opMode);
        } catch (Exception e) {
            return null;
        }
    }

    <S extends AutonomousState> Action setState(Class<S> state, Object... data) {
        return (opMode) -> {
            log("Setting state to " + state.getName());
            var args = new ArrayList<Object>();
            args.add(opMode);
            args.addAll(Arrays.stream(data).collect(Collectors.toList()));
            try {
                opMode.state = state.getConstructor(args.stream().map(o -> o.getClass()).toArray(Class[]::new)).newInstance(args);
            } catch (IllegalAccessException | InstantiationException | InvocationTargetException |
                     NoSuchMethodException e) {
                log("Failed to set state");
                throw new RuntimeException(e);
            }
        };
    }

    Action next() {
        return (opMode) -> {};
    }

    Action stop() {
        return new Action() {
            @Override
            public void accept(CenterStageAutonomous centerStageAutonomous) {

            }

            @Override
            public boolean shouldStop() {
                return true;
            }
        };
    }

    abstract Action loop();
}

public class CenterStageAutonomous extends Linear {
    @Inject
    DriveBase driveBase;

    @Inject
    Vision vision;

    AutonomousState state;

    @Override
    public void opLoop() {
        var result = state.loop();
        result.accept(this);
        if(result.shouldStop()) requestOpModeStop();
    }
}
