define sick_s300 ranger
(
  sensor
  (
    range_max 30.0
    fov 270.0
    samples 1081
  )
  # generic model properties
  color "black"
  size [ 0.106 0.073 0.15 ]
)

define cob4-8 position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  size [ 0.72 0.72 1.58 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "omni"
  color "gray"

  sick_s300(pose [ -0.1 0.0 -0.11 0.0 ])
)
