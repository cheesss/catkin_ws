import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from typing import List

# Import your utility functions and constants
from Source.utilities import (
    plot_3d_lines,
    compute_track_boundaries,
    smooth_reference_path,
    plot_track_boundaries_with_normals,
    plot_sectors_with_boundaries,
    plot_lap_time_history,
)
from Source.pso_optimization import optimize
from Source.cost_functions import (
    calculate_lap_time_with_constraints,
    convert_sectors_to_racing_line,
)
from Source.constants import (
    TRACK_WIDTH,
    DRIVE_WIDTH,
    N_SECTORS,
    N_OPTIMIZABLE_SECTORS,
    N_PARTICLES,
    N_ITERATIONS,
    BOUNDARIES,
    VEHICLE_HEIGHT,
)

def load_reference_path(file_path: str) -> np.ndarray:
    """Load the reference path data from a CSV file."""
    try:
        data = pd.read_csv(file_path, header=None, names=['x', 'y', 'z'])
        reference_path = data[['x', 'y', 'z']].values.astype(float)
        if len(reference_path) == 0:
            raise ValueError("Reference path is empty.")
        return reference_path
    except FileNotFoundError:
        raise RuntimeError(f"File '{file_path}' not found.")
    except Exception as e:
        raise RuntimeError(f"Failed to load reference path: {e}")

def preprocess_reference_path(reference_path: np.ndarray) -> np.ndarray:
    """Smooth the reference path."""
    try:
        return smooth_reference_path(reference_path, smooth_factor=1.0)
    except Exception as e:
        print(f"Warning: {e}. Using original reference path.")
        return reference_path

def initialize_plotting() -> tuple:
    """Initialize real-time plotting."""
    plt.ion()
    fig = plt.figure()
    axis = fig.add_subplot(111, projection='3d')
    plt.title("Real-Time Racing Line Optimization")
    return fig, axis

def update_plot(
    current_solution: List[float],
    iteration: int,
    racing_line_plot,
    axis,
    drive_inside_sectors,
    drive_outside_sectors,
    fixed_start_point,
    fixed_end_point,
) -> None:
    """Update the racing line plot in real-time."""
    try:
        racing_line = convert_sectors_to_racing_line(
            current_solution,
            drive_inside_sectors,
            drive_outside_sectors,
            fixed_start_point,
            fixed_end_point,
        )
        racing_line = np.array(racing_line)
        racing_line_plot.set_data(racing_line[:, 0], racing_line[:, 1])
        racing_line_plot.set_3d_properties(racing_line[:, 2])
        axis.relim()
        axis.autoscale_view()
        plt.draw()
        plt.pause(0.001)
    except Exception as e:
        print(f"Error in update_plot callback: {e}")

def cost_function_wrapper(
    sectors: List[float],
    drive_inside_sectors: np.ndarray,
    drive_outside_sectors: np.ndarray,
    fixed_start_point: np.ndarray,
    fixed_end_point: np.ndarray,
) -> float:
    """Wrapper for the cost function."""
    return calculate_lap_time_with_constraints(
        sectors,
        drive_inside_sectors,
        drive_outside_sectors,
        fixed_start_point,
        fixed_end_point,
    )

def main() -> None:
    """Main function to execute the racing line optimization."""
    try:
        # Load the reference path
        data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Data')
        reference_path_file = os.path.join(data_dir, 'reference_path.csv')
        reference_path = load_reference_path(reference_path_file)

        # Preprocess the reference path
        reference_path = preprocess_reference_path(reference_path)

        # Compute track boundaries
        inside_points, outside_points, normals = compute_track_boundaries(reference_path, TRACK_WIDTH)
        drive_inside_points, drive_outside_points, _ = compute_track_boundaries(reference_path, DRIVE_WIDTH)

        # Offset boundaries for z-axis visualization
        drive_inside_points[:, 2] += VEHICLE_HEIGHT / 2
        drive_outside_points[:, 2] += VEHICLE_HEIGHT / 2

        # Sample reference path into sectors
        sectors_indices = np.linspace(0, len(reference_path) - 1, N_SECTORS, dtype=int)
        drive_inside_sectors = drive_inside_points[sectors_indices]
        drive_outside_sectors = drive_outside_points[sectors_indices]
        mid_sectors = reference_path[sectors_indices]

        # Define fixed start and end points
        fixed_start_point = mid_sectors[0].copy()
        fixed_end_point = mid_sectors[-1].copy()
        fixed_start_point[2] += VEHICLE_HEIGHT / 2
        fixed_end_point[2] += VEHICLE_HEIGHT / 2

        # Initialize plotting
        fig, axis = initialize_plotting()
        racing_line_plot, = axis.plot([], [], [], 'b-', label='Racing Line')
        axis.legend()

        # Define cost function
        cost_function = lambda sectors: cost_function_wrapper(
            sectors,
            drive_inside_sectors,
            drive_outside_sectors,
            fixed_start_point,
            fixed_end_point,
        )

        # Perform PSO optimization
        global_solution, global_evaluation, global_history, evaluation_history = optimize(
            cost_function=cost_function,
            n_dimensions=N_OPTIMIZABLE_SECTORS,
            boundaries=BOUNDARIES,
            n_particles=N_PARTICLES,
            n_iterations=N_ITERATIONS,
            verbose=True,
            callback=lambda sol, it: update_plot(
                sol, it, racing_line_plot, axis, drive_inside_sectors, drive_outside_sectors, fixed_start_point, fixed_end_point
            ),
        )

        # Save optimized racing line
        racing_line = np.array(convert_sectors_to_racing_line(
            global_solution,
            drive_inside_sectors,
            drive_outside_sectors,
            fixed_start_point,
            fixed_end_point,
        ))
        racing_line_file = os.path.join(data_dir, 'racing_line_midpoints.csv')
        pd.DataFrame(racing_line, columns=['x', 'y', 'z']).to_csv(racing_line_file, index=False)
        print(f"Optimized racing line saved to '{racing_line_file}'.")

        # Plot optimization results
        plt.ioff()
        plt.show()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
