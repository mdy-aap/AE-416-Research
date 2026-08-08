/*
========================================================
RESEARCH GANTT CHART
GPS-BASED HIKER TRACKING & MONITORING SYSTEM

DATA SOURCE:
data.json

PROJECT PERIOD:
03 AUGUST 2026 - 22 AUGUST 2026

CURRENT PROGRESS DATE:
10 AUGUST 2026
========================================================
*/


let projectData = null;


/*
========================================================
LOAD DATA.JSON
========================================================
*/

async function loadData() {

    try {

        const response =
            await fetch("data.json");

        if (!response.ok) {

            throw new Error(
                "Unable to load data.json"
            );
        }

        projectData =
            await response.json();

        initialize();

    }

    catch (error) {

        console.error(error);

        document.body.innerHTML = `

            <div style="
                color:white;
                background:#050505;
                min-height:100vh;
                display:flex;
                align-items:center;
                justify-content:center;
                font-family:Arial;
                text-align:center;
                padding:30px;
            ">

                <div>

                    <h1>
                        Unable to load data.json
                    </h1>

                    <p style="
                        color:#888;
                        margin-top:10px;
                    ">
                        Make sure index.html, style.css,
                        script.js and data.json are
                        in the same folder.
                    </p>

                </div>

            </div>

        `;

    }

}


/*
========================================================
INITIALIZE
========================================================
*/

function initialize() {

    displayProjectInformation();

    createDateHeaders();

    createTaskColumn();

    createTimelineGrid();

    createBars();

    createDailyList();

    updateStatistics();

    createTodayLine();

}


/*
========================================================
PROJECT INFORMATION
========================================================
*/

function displayProjectInformation() {

    const project =
        projectData.project;


    document.getElementById(
        "projectTitle"
    ).textContent =
        project.title;


    document.getElementById(
        "projectSubtitle"
    ).textContent =
        project.subtitle;


    document.getElementById(
        "chapterTitle"
    ).textContent =
        project.chapter;


    const start =
        formatDate(project.startDate);


    const end =
        formatDate(project.endDate);


    document.getElementById(
        "projectPeriod"
    ).textContent =
        `${start} — ${end}`;


    document.getElementById(
        "currentDate"
    ).textContent =
        formatDate(project.currentDate);

}


/*
========================================================
FORMAT DATE
========================================================
*/

function formatDate(dateString) {

    const date =
        new Date(
            dateString + "T00:00:00"
        );

    const day =
        String(
            date.getDate()
        ).padStart(2, "0");


    const month =
        date.toLocaleString(
            "en-US",
            {
                month: "short"
            }
        ).toUpperCase();


    const year =
        date.getFullYear();


    return `${day} ${month} ${year}`;

}


/*
========================================================
GET PROJECT DATES
========================================================
*/

function getDates() {

    const start =
        new Date(
            projectData.project.startDate
            + "T00:00:00"
        );


    const end =
        new Date(
            projectData.project.endDate
            + "T00:00:00"
        );


    const dates = [];


    let current =
        new Date(start);


    while(current <= end) {

        dates.push(
            new Date(current)
        );

        current.setDate(
            current.getDate() + 1
        );

    }


    return dates;

}


/*
========================================================
DATE HEADERS
========================================================
*/

function createDateHeaders() {

    const container =
        document.getElementById(
            "dates"
        );


    const dates =
        getDates();


    const currentDate =
        new Date(
            projectData.project.currentDate
            + "T00:00:00"
        );


    dates.forEach(date => {

        const element =
            document.createElement(
                "div"
            );


        element.className =
            "date";


        const day =
            String(
                date.getDate()
            ).padStart(2, "0");


        element.textContent =
            day;


        if(
            date.getTime() ===
            currentDate.getTime()
        ) {

            element.classList.add(
                "today"
            );

        }


        container.appendChild(
            element
        );

    });

}


/*
========================================================
TASK COLUMN
========================================================
*/

function createTaskColumn() {

    const container =
        document.getElementById(
            "taskColumn"
        );


    projectData.tasks.forEach(
        (task, index) => {

            const row =
                document.createElement(
                    "div"
                );


            row.className =
                "task";


            row.innerHTML = `

                <strong>
                    ${String(index + 1)
                        .padStart(2, "0")}
                </strong>

                <span>
                    ${task.task}
                </span>

            `;


            container.appendChild(
                row
            );

        }
    );

}


/*
========================================================
TIMELINE GRID
========================================================
*/

function createTimelineGrid() {

    const container =
        document.getElementById(
            "timelineGrid"
        );


    projectData.tasks.forEach(
        () => {

            const row =
                document.createElement(
                    "div"
                );


            row.className =
                "grid-row";


            const dates =
                getDates();


            dates.forEach(
                () => {

                    const cell =
                        document.createElement(
                            "div"
                        );


                    cell.className =
                        "grid-cell";


                    row.appendChild(
                        cell
                    );

                }
            );


            container.appendChild(
                row
            );

        }
    );

}


/*
========================================================
GANTT BARS
========================================================
*/

function createBars() {

    const container =
        document.getElementById(
            "bars"
        );


    projectData.tasks.forEach(
        (task, index) => {

            const bar =
                document.createElement(
                    "div"
                );


            bar.className =
                `bar ${task.status}`;


            bar.style.setProperty(
                "--start",
                index + 1
            );


            bar.style.setProperty(
                "--width",
                1
            );


            bar.textContent =
                `${task.progress}%`;


            bar.title =
                `${task.task} — ${task.progress}%`;


            container.appendChild(
                bar
            );

        }
    );

}


/*
========================================================
DAILY PROGRESS
========================================================
*/

function createDailyList() {

    const container =
        document.getElementById(
            "dailyList"
        );


    projectData.tasks.forEach(
        task => {

            const item =
                document.createElement(
                    "div"
                );


            item.className =
                `daily-item ${task.status}`;


            item.innerHTML = `

                <div class="daily-left">

                    <div class="daily-date">
                        ${formatShortDate(task.date)}
                    </div>

                    <div class="daily-task">
                        ${task.task}
                    </div>

                </div>

                <div class="daily-percent">
                    ${task.progress}%
                </div>

            `;


            container.appendChild(
                item
            );

        }
    );

}


/*
========================================================
SHORT DATE
========================================================
*/

function formatShortDate(
    dateString
) {

    const date =
        new Date(
            dateString + "T00:00:00"
        );


    const day =
        String(
            date.getDate()
        ).padStart(2, "0");


    const month =
        date.toLocaleString(
            "en-US",
            {
                month: "short"
            }
        );


    return `${day} ${month}`;

}


/*
========================================================
STATISTICS
========================================================
*/

function updateStatistics() {

    const tasks =
        projectData.tasks;


    const completed =
        tasks.filter(
            task =>
                task.progress >= 80
        ).length;


    const current =
        tasks.filter(
            task =>
                task.status === "current"
        ).length;


    const partial =
        tasks.filter(
            task =>
                task.progress > 0 &&
                task.progress < 80
        ).length;


    const totalProgress =
        tasks.reduce(
            (sum, task) =>
                sum + task.progress,
            0
        );


    const overall =
        Math.round(
            totalProgress /
            tasks.length
        );


    document.getElementById(
        "completedCount"
    ).textContent =
        completed;


    document.getElementById(
        "progressCount"
    ).textContent =
        current;


    document.getElementById(
        "partialCount"
    ).textContent =
        partial;


    document.getElementById(
        "overallProgress"
    ).textContent =
        overall + "%";

}


/*
========================================================
CURRENT DATE LINE
========================================================
*/

function createTodayLine() {

    const timeline =
        document.querySelector(
            ".timeline"
        );


    const line =
        document.createElement(
            "div"
        );


    line.className =
        "today-line";


    const dates =
        getDates();


    const currentDate =
        new Date(
            projectData.project.currentDate
            + "T00:00:00"
        );


    const index =
        dates.findIndex(
            date =>
                date.getTime() ===
                currentDate.getTime()
        );


    if(index >= 0) {

        line.style.left =
            `${(index / dates.length) * 100}%`;

    }


    timeline.appendChild(
        line
    );

}


/*
========================================================
START APPLICATION
========================================================
*/

loadData();
