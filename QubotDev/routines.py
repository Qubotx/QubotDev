def define_routines():
    """
    Define movement routines as sequences of (linear_speed, angular_speed, duration)
    Converted from modo_espera.py RPM-based routines

    Robot parameters used for conversion:
    - wheel_radius = 0.065 m
    - wheel_base = 0.38 m
    """

    # Basic movement primitives
    basic_moves = {
        # straight forward at 0.136 m/s for 2s
        "forward": [(0.136, 0.0, 2.0)],
        # straight backward at 0.136 m/s for 2s
        "backward": [(-0.136, 0.0, 2.0)],
        # rotate right 45° (π/4 rad) in 0.5s
        "right": [(0.0, -1.571, 0.5)],
        # rotate left 45° (π/4 rad) in 0.5s
        "left": [(0.0, 1.571, 0.5)],
        # rotate right 90° (π/2 rad) in 1s
        "cuarto_giro_derecha": [(0.0, -1.571, 1)],
        # rotate left 90° (π/2 rad) in 1s
        "cuarto_giro_izquierda": [(0.0, 1.571, 1)],
        # rotate right 180° (π rad) in 2s
        "medio_giro_derecha": [(0.0, -1.571, 2)],
        # rotate left 180° (π rad) in 2s
        "medio_giro_izquierda": [(0.0, 1.571, 2)],
        # rotate right 360° (2π rad) in 4s
        "giro_completo_derecha": [(0.0, -1.571, 4)],
        # rotate left 360° (2π rad) in 4s
        "giro_completo_izquierda": [(0.0, 1.571, 4)],
        # curved path: forward 0.113 m/s, turning right 0.567 rad/s for 5s
        "circulo_derecha": [(0.113, -0.567, 5.0)],
        # curved path: forward 0.113 m/s, turning left 0.567 rad/s for 5s
        "circulo_izquierda": [(0.113, 0.567, 5.0)],
        # Wait/pause commands
        "wait_short": [(0.0, 0.0, 0.5)],
        "wait_medium": [(0.0, 0.0, 1.0)],
        "wait_long": [(0.0, 0.0, 1.5)],
    }

    # Helper function to combine routines
    def combine_routines(*routine_names):
        """Combine multiple routines into one sequence"""
        combined = []
        for name in routine_names:
            if name in basic_moves:
                combined.extend(basic_moves[name])
            else:
                print(f"Warning: Routine '{name}' not found in basic_moves")
        return combined

    # Build complex routines using basic moves
    routines = basic_moves.copy()  # Include all basic moves

    # Complex routines built from basic moves
    routines["rutina_paseo"] = combine_routines(
        # First section: 3 forward + turn
        *["forward"] * 3,
        "wait_short",
        "medio_giro_izquierda",
        "wait_short",
        # Second section: 4 forward + turn
        *["forward"] * 4,
        "wait_short",
        "medio_giro_izquierda",
        "wait_short",
        # Third section: 2 forward + turn + spin
        *["forward"] * 2,
        "wait_medium",
        "medio_giro_izquierda",
        "wait_medium",
        "giro_completo_derecha",
        "wait_short",
        # Fourth section: forward + spin
        "forward",
        "wait_long",
        "giro_completo_izquierda",
        "wait_short",
        # Fifth section: 2 forward + turn + circles
        *["forward"] * 2,
        "wait_medium",
        "medio_giro_izquierda",
        "wait_short",
        "circulo_derecha",
        "wait_long",
        "giro_completo_izquierda",
        "wait_medium",
        "circulo_izquierda",
        "wait_short",
        "medio_giro_izquierda",
        "wait_short",
        # Final section: 3 forward + final moves
        *["forward"] * 3,
        "wait_medium",
        "medio_giro_derecha",
        "wait_short",
        "forward",
        "wait_medium",
        "giro_completo_derecha",
    )

    # Example of other complex routines you can easily create
    routines["simple_square"] = combine_routines(
        "forward",
        "cuarto_giro_izquierda",
        "forward",
        "cuarto_giro_izquierda",
        "forward",
        "cuarto_giro_izquierda",
        "forward",
        "cuarto_giro_izquierda",
    )

    routines["figure_eight"] = combine_routines("circulo_derecha", "circulo_izquierda")

    routines["back_and_forth"] = combine_routines(
        *["forward"] * 3,
        "giro_completo_derecha",
        *["forward"] * 3,
        "giro_completo_derecha",
    )

    routines["routine1"] = [
        (0.3, 0.0, 1.5),  # move straight at 0.3m/s for 1.5s
        (0.3, 0.2, 0.5),  # curve at 0.3m/s with 0.2rad/s for 0.5s
        (0.0, -0.5, 3.0),  # rotate in place at -0.5rad/s for 3s
    ]

    routines["test"] = [
        (0.5, 0.0, 3.0),  # forward
        (-0.5, 0.0, 3.0),  # backward
        (0.5, 0.0, 3.0),  # forward
        (-0.5, 0.0, 3.0),  # backward
        (0.5, 0.0, 3.0),  # forward
        (-0.5, 0.0, 3.0),  # backward
    ]

    routines["test_forward"] = [
        (0.5, 0.0, 3.0),  # forward
        (0.0, 0.0, 1.0),  # pause
        (0.5, 0.0, 3.0),  # forward
        (0.0, 0.0, 1.0),  # pause
        (0.5, 0.0, 3.0),  # forward
        (0.0, 0.0, 1.0),  # pause
        (0.5, 0.0, 3.0),  # forward
        (0.0, 0.0, 1.0),  # pause
        (0.5, 0.0, 3.0),  # forward
        (0.0, 0.0, 1.0),  # pause
        (0.5, 0.0, 3.0),  # forward
        (0.0, 0.0, 1.0),  # pause
        (0.5, 0.0, 3.0),  # forward
    ]

    return routines


# # Additional utility functions
# def create_custom_routine(self, routine_definition):
#     """
#     Create a custom routine from a list of basic move names

#     Example usage:
#     custom = create_custom_routine([
#         "forward", "forward",
#         "medio_giro_izquierda",
#         "circulo_derecha"
#     ])
#     """
#     basic_moves = self.define_routines()
#     combined = []
#     for move_name in routine_definition:
#         if move_name in basic_moves:
#             combined.extend(basic_moves[move_name])
#         else:
#             print(f"Warning: Move '{move_name}' not found")
#     return combined


# def repeat_sequence(self, sequence, times):
#     """
#     Repeat a sequence of moves multiple times

#     Example:
#     patrol = repeat_sequence(["forward", "medio_giro_derecha"], 4)
#     """
#     repeated = []
#     for _ in range(times):
#         repeated.extend(self.create_custom_routine(sequence))
#     return repeated
