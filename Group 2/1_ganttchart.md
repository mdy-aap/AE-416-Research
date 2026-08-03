```mermaid
gantt
    title Chapter 3: Data Gathering Procedures
    dateFormat  YYYY-MM-DD
    axisFormat  %b %d

    section Geometric Characterization & BEMT
    3.1 Propeller Geometry & BEMT Setup     :active, c3_1, 2026-08-08, 4d
    BEMT Theoretical Model Validated        :milestone, m1, 2026-08-11, 0d

    section Test Rig Build & DAQ Setup
    3.2 Test Bench Rig & Load Cell Assembly :c3_2, 2026-08-12, 5d
    3.3 Sensor Calibration & DAQ Testing    :c3_3, 2026-08-17, 3d
    Test Rig & Sensors Calibrated           :milestone, m2, 2026-08-19, 0d

    section Methodology Manuscript
    3.4 Write Chapter 3 Methodology Draft   :crit, c3_4, 2026-08-20, 3d
    Chapter 3 Initial Draft Completed       :milestone, m3, 2026-08-22, 0d
