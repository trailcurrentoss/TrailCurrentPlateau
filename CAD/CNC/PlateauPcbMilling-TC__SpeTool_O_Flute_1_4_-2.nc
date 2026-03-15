(Exported by FreeCAD)
(Post Processor: grbl_post)
(Output Time:2026-03-15 17:49:05.879577)
(Begin preamble)
G17 G90
G21
(Begin operation: TC: SpeTool O Flute 1/4")
(Path: TC: SpeTool O Flute 1/4")
(TC: SpeTool O Flute 1/4")
(Begin toolchange)
( M6 T3 )
M3 S18000
(Finish operation: TC: SpeTool O Flute 1/4")
(Begin operation: Fixture)
(Path: Fixture)
G54
(Finish operation: Fixture)
(Begin operation: ProfileCutEdges)
(Path: ProfileCutEdges)
(ProfileCutEdges)
(Compensated Tool Path. Diameter: 6.35)
G0 Z5.000
G0 X70.342 Y60.210
G0 Z3.000
G1 X70.342 Y60.210 Z-1.620 F200.000
G2 X71.272 Y57.964 Z-1.620 I-2.245 J-2.245 K0.000 F400.000
G1 X71.272 Y0.024 Z-1.620 F400.000
G2 X68.097 Y-3.151 Z-1.620 I-3.175 J0.000 K0.000 F400.000
G1 X0.025 Y-3.151 Z-1.620 F400.000
G2 X-3.150 Y0.024 Z-1.620 I0.000 J3.175 K0.000 F400.000
G1 X-3.150 Y57.965 Z-1.620 F400.000
G2 X0.025 Y61.139 Z-1.620 I3.175 J-0.000 K0.000 F400.000
G1 X68.097 Y61.139 Z-1.620 F400.000
G2 X70.342 Y60.210 Z-1.620 I-0.000 J-3.175 K0.000 F400.000
G0 Z5.000
G0 Z5.000
(Finish operation: ProfileCutEdges)
(Begin postamble)
M5
G17 G90
M2
