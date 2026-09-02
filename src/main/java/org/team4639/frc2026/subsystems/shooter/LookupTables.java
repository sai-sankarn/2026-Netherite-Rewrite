package org.team4639.frc2026.subsystems.shooter;

import java.util.AbstractMap;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class LookupTables {
    public static final double MIN_SCORING_RPM = 2320.0;
    public static final double MAX_SCORING_RPM = 3765.0;

    public static final InterpolatingDoubleTreeMap scoringDistanceToRPM = InterpolatingDoubleTreeMap.ofEntries(
            new AbstractMap.SimpleImmutableEntry<>(1.87, MIN_SCORING_RPM),
            new AbstractMap.SimpleImmutableEntry<>(2.20, 2520.0),
            new AbstractMap.SimpleImmutableEntry<>(2.44, 2690.0),
            new AbstractMap.SimpleImmutableEntry<>(2.90, 2825.0),
            new AbstractMap.SimpleImmutableEntry<>(3.20, 2930.0),
            new AbstractMap.SimpleImmutableEntry<>(3.50, 3015.0),
            new AbstractMap.SimpleImmutableEntry<>(3.80, 3100.0),
            new AbstractMap.SimpleImmutableEntry<>(4.10, 3260.0),
            new AbstractMap.SimpleImmutableEntry<>(4.41, 3370.0),
            new AbstractMap.SimpleImmutableEntry<>(4.77, 3465.0),
            new AbstractMap.SimpleImmutableEntry<>(4.90, 3635.0),
            new AbstractMap.SimpleImmutableEntry<>(5.20, MAX_SCORING_RPM));
}
