```mermaid
gantt
    title Group 2
    dateFormat  YYYY-MM-DD
    axisFormat  %b %d

    section Research Paper
    Chapter 1       :done,     des1, 2026-07-03, 2026-07-25
    Chapter 2       :done,     des2, 2026-07-03, 2026-07-25
    Chapter 3       :crit,   des3, 2026-08-03, 10d
    Chapter 4       :active, des4

    section Development
    Backend Setup         :active,  dev1, 2026-09-10, 10d
    Frontend Integration  :         dev2, after des3, 8d
    Database Migration    :crit,    dev3, after dev1, 4d

    section Testing & Deployment
    QA Testing            :         qa1,  after dev2, 5d
    Final Deployment      :milestone, m1, after qa1, 0d
```
