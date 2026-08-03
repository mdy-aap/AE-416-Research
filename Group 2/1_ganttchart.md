```mermaid
gantt
    Title BEMT and Ground-Based Static Propeller Testing Schedule
    dateFormat  YYYY-MM-DD
    axisFormat  %b %d

    section Prelim Term - Completed
    Chapters 1 and 2 Drafting             :done, ch12, 2026-07-03, 2026-07-25
    Defense Milestone               :milestone, m_title, 2026-07-25, 0d

    section Phase 1: BEMT and Geometry
    1.1 Propeller Geometric Characterization :active, p1_1, 2026-08-03, 7d
    1.2 AeroSandbox BEMT Python Code         :p1_2, 2026-08-10, 7d

    section Phase 2: Hardware and DAQ Rig
    2.1 Test Bench Mechanical Build          :p2_1, 2026-08-17, 10d
    2.2 ESP32-S3 and Load Cell Integration   :p2_2, 2026-08-27, 6d
    2.3 Sensor Calibration and Linearity     :p2_3, 2026-09-02, 5d

    section Phase 3: Chapter 3 and Pre-Final
    3.1 Write Chapter 3 - Data Gathering     :crit, p3_1, 2026-09-07, 12d
    3.2 Manuscript Assembly and Deck         :crit, p3_2, 2026-09-19, 9d
    Pre-Final Research Defense               :milestone, m_prefinal, 2026-09-28, 0d

    section Phase 4: Static Bench Runs
    4.1 Static RPM Testing and Telemetry     :p4_1, 2026-09-29, 9d

    section Phase 5: Statistics and Chapter 4
    5.1 Statistical Processing and Error Analysis :p5_1, 2026-10-08, 6d
    5.2 Write Chapter 4 - Results and Discussion   :crit, p5_2, 2026-10-14, 8d

    section Phase 6: Final SOP and Submission
    6.1 AAP Drone Club Workflow Manual       :p6_1, 2026-10-18, 7d
    6.2 Final Manuscript Assembly and Deck   :crit, p6_2, 2026-10-22, 4d
    Final Research Defense                   :milestone, m_final, 2026-10-26, 0d
    6.3 Post-Defense Edits and Hardbound     :p6_3, 2026-10-27, 8d
