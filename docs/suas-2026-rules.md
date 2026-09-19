# SUAS 2026 rules, boiled down

Source: https://robonation.gitbook.io/suas-resources/2026-team-handbook. We use these as a guideline for 2027; expect the theme, targets, and payloads to change when the 2027 handbook lands.
Event: Sep 14-17 2026, Skyway Range, Tulsa OK. Theme: storm response.
Scoring (mission 1000): Flight Endurance 250, Efficient Operators 200, Rapid Response 200, Risk Mapping 150, Search/Detect/Deliver 200. Design docs 250.
Flight Endurance: lap ~2 mi, <=15 waypoints, within 100 ft; 1-10 laps; 200*(N/10)^2 + 20 takeoff/land + 30 if autonomous. Takeoff success = above 150 ft AGL.
Flight perf: 100 ft wp error, 150 ft turn radius, 20 deg climb/descent, altitude band 150-400 ft AGL (below 150 only takeoff/landing within 500 ft of runway).
Runways: R1 40x500 ft paved (VTOL+HTOL), R2 40x40 ft (VTOL only). Field 633-748 ft MSL, runway ~690. Trees up to 100 ft AGL. Mag dev 2 W.
Flight boundary: 11-pt polygon (~36.2106-36.2208 N, -96.0017 to -96.0142 W) ≈ 1.1 km x 1.1 km.
Search boundaries: two ~10 acre quads. SB1 (36.216341,-96.010424)(36.21675,-96.00755)(36.218054,-96.007835)(36.217645,-96.010709). SB2 (36.213469,-96.002635)(36.215795,-96.002635)(36.215795,-96.004286)(36.213469,-96.004286).
Search/Detect/Deliver: targets = mannequin (any orientation, near bushes/trees/vehicles) -> 8oz water bottle (~255 g); pop-up tent (open) -> GP908 strobe beacon (~155 g). Carry all payloads at once. Min alt 150 ft AGL. Must fly >=1 lap first. No freefall (needs retardant, e.g. parachute). Per delivery 100: 20 survives in SB, 50 within 50 ft, 30 correct match. Payload <=2 lb, no propulsion.
Risk Mapping: stitched orthomosaic PNG/JPEG of a search boundary, google-maps quality; 0-150 sliding; submit via USB within mission time.
Efficient Operators: 200*min(1,(4-O)/2). Rapid Response: weight 50*min(1,(35-W)/20); volume tiers (personal item 50 / carry-on 25 / check-in 10); tool-less 3 min assembly 25; batteries all <100 Wh 75.
UAS: <=35 lb, RTL failsafe, flight termination, remote ID, wind ~14 mph, up to 110F. Mission time 45 min setup+mission (30 min mission window for map).
Penalties: excess time 0.5%/s; things fall off 10%/item; crash 50%; unsafe ops 50%; manual takeover -> restart lap; unsafe OOB -> terminate.
