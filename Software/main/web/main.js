const units = [
    {
        id: "ms",
        label: "m/s",
        valueKey: "windSpeedMs",
        unitSuffix: "m/s",
    },
    {
        id: "kmh",
        label: "km/h",
        valueKey: "windSpeedKmh",
        unitSuffix: "km/h",
    },
    {
        id: "pa",
        label: "Pa",
        valueKey: "pressureDiffPa",
        unitSuffix: "Pa",
    },
];

let selectedUnitIndex = 0;

const speedValueEl = document.getElementById("speed-value");
const speedUnitEl = document.getElementById("speed-unit");
const angleValueEl = document.getElementById("angle-value");
const compassValueEl = document.getElementById("compass-value");
const unitToggleButton = document.getElementById("unit-toggle");

function formatNumeric(value) {
    if (typeof value !== "number" || Number.isNaN(value)) {
        return "--";
    }
    return value.toFixed(2);
}

function applyUnitToUi(data) {
    const unit = units[selectedUnitIndex];
    const rawValue = data[unit.valueKey];

    speedValueEl.textContent = formatNumeric(rawValue);
    speedUnitEl.textContent = unit.unitSuffix;
    unitToggleButton.textContent = `Unit: ${unit.label}`;
}

function applyAngleToUi(data) {
    const hasWind = Boolean(data.hasWind);
    const angle = Number(data.relativeAngleDeg);

    if (!hasWind || Number.isNaN(angle) || angle < 0) {
        angleValueEl.textContent = "--";
        return;
    }

    angleValueEl.textContent = `${angle}`;
}

function applyCompassToUi(data) {
    const heading = Number(data.compassHeadingDeg);

    if (Number.isNaN(heading) || heading < 0) {
        compassValueEl.textContent = "--";
        return;
    }

    compassValueEl.textContent = `${Math.round(heading)}`;
}

async function refreshData() {
    try {
        const response = await fetch("/data", { cache: "no-store" });
        const data = await response.json();

        if (!response.ok) {
            throw new Error(data.error || "Failed to fetch data");
        }

        applyUnitToUi(data);
        applyAngleToUi(data);
        applyCompassToUi(data);
    } catch (error) {
        console.error(error.message || "Failed to fetch telemetry");
    }
}

unitToggleButton.addEventListener("click", () => {
    selectedUnitIndex = (selectedUnitIndex + 1) % units.length;
    refreshData();
});

refreshData();
setInterval(refreshData, 1000);
