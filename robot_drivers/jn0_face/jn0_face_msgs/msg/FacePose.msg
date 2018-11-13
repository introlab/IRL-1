# NOTE: This is the lower-level command format.
# See FaceExpression for user interface.
#
# All values are clamped between -1 and 1.
# This range goes over the admissible range of each facial component.
# Currently, all servos are limited [-40, +40] degrees.
# NOTE: To prevent collisions, an extra limit is applied to mouth positions.

float32 left_eye_pan
float32 right_eye_pan
float32 eyes_tilt
float32 right_brow
float32 left_brow
float32 top_left_mouth
float32 bottom_left_mouth
float32 top_right_mouth
float32 bottom_right_mouth
