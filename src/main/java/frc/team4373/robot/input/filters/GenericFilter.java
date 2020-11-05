package frc.team4373.robot.input.filters;

/**
 * Generic Filter interface
 * All filters shall implement this interface
 * based on a specific type.
 */

public interface GenericFilter<E> {
    E applyFilter(E input);
}