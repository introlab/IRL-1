# Pose vector (8x1) members order:
#  - eyebrow_left
#  - eyebrow_right
#  - mouth_top_left
#  - mouth_top_right
#  - mouth_bottom_left
#  - mouth_bottom_right
#  - head_bias_pan
#  - head_bias_tilt

float64[8]  pose    # Mean pose.
float64[8]  drift   # Pose drift for the randomizer.
duration    period  # Cycling period. 

