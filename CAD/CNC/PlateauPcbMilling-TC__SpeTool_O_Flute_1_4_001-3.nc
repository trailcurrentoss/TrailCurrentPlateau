(Exported by FreeCAD)
(Post Processor: grbl_post)
(Output Time:2026-03-16 21:47:24.824213)
(Begin preamble)
G17 G90
G21
(Begin operation: TC: SpeTool O Flute 1/4"001)
(Path: TC: SpeTool O Flute 1/4"001)
(TC: SpeTool O Flute 1/4"001)
(Begin toolchange)
( M6 T4 )
M3 S18000
(Finish operation: TC: SpeTool O Flute 1/4"001)
(Begin operation: Fixture)
(Path: Fixture)
G54
(Finish operation: Fixture)
(Begin operation: ProfileEdgeCuts)
(Path: ProfileEdgeCuts)
(ProfileEdgeCuts)
(Compensated Tool Path. Diameter: 6.35)
G0 Z5.020
G0 X70.317 Y60.185
G0 Z3.020
G1 X70.317 Y60.185 Z-2.500 F200.000
G2 X71.247 Y57.940 Z-2.500 I-2.245 J-2.245 K0.000 F400.000
G1 X71.247 Y0.000 Z-2.500 F400.000
G2 X68.072 Y-3.175 Z-2.500 I-3.175 J0.000 K0.000 F400.000
G1 X0.000 Y-3.175 Z-2.500 F400.000
G2 X-3.175 Y0.000 Z-2.500 I0.000 J3.175 K0.000 F400.000
G1 X-3.175 Y57.940 Z-2.500 F400.000
G2 X0.000 Y61.115 Z-2.500 I3.175 J-0.000 K0.000 F400.000
G1 X68.072 Y61.115 Z-2.500 F400.000
G2 X70.317 Y60.185 Z-2.500 I-0.000 J-3.175 K0.000 F400.000
G0 Z5.020
G0 Z5.020
(Finish operation: ProfileEdgeCuts)
(Begin postamble)
M5
G17 G90
M2
