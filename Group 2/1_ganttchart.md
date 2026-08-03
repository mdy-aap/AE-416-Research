```mermaid
gantt
    title BEMT and Ground-Based Static Propeller Testing Schedule
    dateFormat  YYYY-MM-DD
    axisFormat  %b %d

    section Ch. 1 - Completed
    1.1 Intro                            :done, c1_1, 2026-07-05, 1d
    1.2 Statement of Problem             :done, c1_2, 2026-07-06, 1d
    1.3 Objectives                       :done, c1_3, 2026-07-07, 1d
    1.4 Significance                     :done, c1_4, 2026-07-08, 1d
    1.5 Scope & Limitations              :done, c1_5, 2026-07-09, 1d
    1.6 Theoretical Framework            :done, c1_6, 2026-07-10, 1d
    1.7 Conceptual Framework             :done, c1_7, 2026-07-11, 1d
    1.8 Definition of Terms              :done, c1_8, 2026-07-12, 1d

    section Ch. 2 - Completed
    2.1 Research Design & Method         :done, c2_1, 2026-07-13, 1d
    2.2 Data Gathering - Digital         :done, c2_2, 2026-07-14, 1d
    2.3 Data Gathering - Physical        :done, c2_3, 2026-07-15, 1d
    2.4 Statistical Treatment            :done, c2_4, 2026-07-16, 1d
    2.5 Format & Master Check            :done, c2_5, 2026-07-17, 1d
    2.6 Defense Deck & Rehearsal         :done, c2_6, 2026-07-18, 2d
    Title Defense Milestone              :milestone, m_title, 2026-07-25, 0d

    section Phase 1: BEMT & Geometry
    1.1 Propeller Geometry Setup         :active, p1_1, 2026-08-03, 7d
    1.2 AeroSandbox BEMT Python Code     :p1_2, 2026-08-10, 7d

    section Phase 2: Hardware & DAQ Rig
    2.1 Test Bench Mechanical Build      :p2_1, 2026-08-17, 10d
    2.2 ESP32-S3 & Load Cell Integration :p2_2, 2026-08-27, 6d
    2.3 Sensor Calibration & Linearity   :p2_3, 2026-09-02, 5d

    section Phase 3: Chapter 3 & Pre-Final
    3.1 Write Chapter 3 - Data Gathering :crit, p3_1, 2026-09-07, 12d
    3.2 Manuscript Assembly & Deck       :crit, p3_2, 2026-09-19, 9d
    Pre-Final Research Defense           :milestone, m_prefinal, 2026-09-28, 0d

    section Phase 4: Static Bench Runs
    4.1 Static RPM Testing & Telemetry   :p4_1, 2026-09-29, 9d

    section Phase 5: Statistics & Chapter 4
    5.1 Statistical Processing & Errors  :p5_1, 2026-10-08, 6d
    5.2 Write Chapter 4 - Discussion     :crit, p5_2, 2026-10-14, 8d

    section Phase 6: Final SOP & Submission
    6.1 AAP Drone Club Workflow Manual   :p6_1, 2026-10-18, 7d
    6.2 Final Manuscript Assembly & Deck :crit, p6_2, 2026-10-22, 4d
    Final Research Defense               :milestone, m_final, 2026-10-26, 0d
    6.3 Post-Defense Edits & Hardbound   :p6_3, 2026-10-27, 8d
