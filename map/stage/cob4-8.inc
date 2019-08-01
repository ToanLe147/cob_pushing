define sick_s300 ranger
(
  sensor
  (
    range_max 29.5
    fov 270.0
    samples 541
  )
  # generic model properties
  color "black"
  size [ 0.106 0.073 0.15 ]
)

define cob4-8 position
(
  block
  (
    points 6
    point[0] [0.4 0]
    point[1] [0.35 0.35]
    point[2] [-0.2 0.4]
    point[3] [-0.45 0]
    point[4] [-0.2 -0.4]
    point[5] [0.35 -0.35]
    z [0 1.58]
  )

  pose [ 0.0 0.0 0.0 0.0 ]

  size [ 0.72 0.72 1.58 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "omni"
  color "gray"

  sick_s300(pose [ 0.244 0.0 -1.37142 0.0 ])
  sick_s300(pose [ 0.122 0.21131 -1.37142 120.0 ])
  sick_s300(pose [ 0.122 -0.21131 -1.37142 -120.0 ])
)
