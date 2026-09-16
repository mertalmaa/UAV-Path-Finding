# Low-flight production validation

Endpoint mode: fixed; target AGL: 100 m; hard minimum: 100 m.

Search and altitude refinement use production modules. The altitude solver finds the lowest feasible profile on the selected ground track; it does not prove a globally optimal 3D route.

| Mission | Result | Mean AGL before / after (m) | Verified minimum (m) | Max climb / descent (m/s) |
|---|---|---:|---:|---:|
| B1_Mountain_Circumnavigation | FOUND | 201.4 / 157.3 | 100.0 | 5.00 / 5.00 |
| B2_Left_Right_Bypass | FOUND | 214.7 / 176.5 | 100.0 | 5.00 / 5.00 |
| B3_Ridge_Crossing_Chosen | FOUND | 332.2 / 295.8 | 176.3 | 5.00 / 5.00 |
| B4_Ridge_Too_High_Detour | FOUND | 201.4 / 157.3 | 100.0 | 5.00 / 5.00 |
| B5_S_Shaped_Corridor | FOUND | 187.0 / 170.5 | 100.0 | 5.00 / 5.00 |
| B6_Narrow_Valley_Turn | FOUND | 128.0 / 114.7 | 100.0 | 5.00 / 5.00 |
