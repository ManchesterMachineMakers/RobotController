package org.firstinspires.ftc.teamcode.diagnostics.tests;

import org.firstinspires.ftc.teamcode.diagnostics.util.Testable;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface RequirementsDeclaration {
    Requires[] value();
}
