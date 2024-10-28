package org.firstinspires.ftc.teamcode.noncents;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public class Util {
    public static <E> Set<E> setOf(E... list) {
        return Collections.unmodifiableSet(new HashSet<>(Arrays.asList(list)));
    }
}
