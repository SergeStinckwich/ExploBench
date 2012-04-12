include "sick.inc"

define segway position
(
  color "gray"
  size [0.7  0.8 1.75]
  origin [0 0 0 0]
  gui_nose 1
  drive "diff"

  localization "gps"

  sick(pose [0.2 0.0 0.0 0.67])
)
