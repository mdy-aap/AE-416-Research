```mermaid
gantt
    title Group 2 BEMT Thrust & Torque Test Stand
    dateFormat  YYYY-MM-DD
    axisFormat  %b %d

    section Research Paper
    Chapter 1            :done,   des, 2026-07-03, 2026-07-25
    Chapter 2            :done,   des, 2026-07-03, 2026-07-25
    Chapter 3            :crit,   des, 2026-08-03, 10d
    Chapter 4            :active, des, after des, 1d

    section Development
    Electronics Testing  :crit,       dev, 2026-07-20, 3d
    Code                 :crit,       dev, 2026-07-20, 4d
    Test Rig             :milestone,  dev, after des3, 16d

    section Testing
    First Testing        :milestone, dev, 2026-08-03, 1d
    Optimization         :active,    dev, 2026-08-03, 10d
    Final Testing        :milestone, dev, 2026-08-03, 0d

    section Presentation
    idk                  :active, group, 2026-08-24, 0d
```
