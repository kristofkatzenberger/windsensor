let currentPath = "/";

function clearNode(node) {
    while (node.firstChild) {
        node.removeChild(node.firstChild);
    }
}

function formatSize(bytes) {
    if (!bytes) {
        return "File";
    }

    const units = ["B", "KB", "MB", "GB"];
    let value = bytes;
    let unit = 0;

    while (value >= 1024 && unit < units.length - 1) {
        value /= 1024;
        unit += 1;
    }

    return `${unit === 0 ? value : value.toFixed(1)} ${units[unit]}`;
}

function getParentPath(path) {
    if (path === "/" || !path) {
        return "/";
    }

    const parts = path.split("/").filter(Boolean);
    parts.pop();
    return parts.length ? `/${parts.join("/")}` : "/";
}

function makeButton(label, className, onClick) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = className;
    button.textContent = label;
    button.addEventListener("click", onClick);
    return button;
}

function makeIconLink(className, href, title, iconMarkup) {
    const link = document.createElement("a");
    link.className = className;
    link.href = href;
    link.title = title;
    link.setAttribute("aria-label", title);
    link.innerHTML = iconMarkup;
    return link;
}

function makeIconButton(className, title, iconMarkup, onClick) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = className;
    button.title = title;
    button.setAttribute("aria-label", title);
    button.innerHTML = iconMarkup;
    button.addEventListener("click", onClick);
    return button;
}

const downloadIcon = '<svg viewBox="0 0 24 24" aria-hidden="true" focusable="false"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"></path><polyline points="7 10 12 15 17 10"></polyline><line x1="12" y1="15" x2="12" y2="3"></line></svg>';
const trashIcon = '<svg viewBox="0 0 24 24" aria-hidden="true" focusable="false"><polyline points="3 6 5 6 21 6"></polyline><path d="M19 6l-1 14a2 2 0 0 1-2 2H8a2 2 0 0 1-2-2L5 6"></path><path d="M10 11v6"></path><path d="M14 11v6"></path><path d="M9 6V4a1 1 0 0 1 1-1h4a1 1 0 0 1 1 1v2"></path></svg>';

async function deleteFile(entry) {
    const confirmed = window.confirm(`Delete ${entry.name}?`);

    if (!confirmed) {
        return;
    }

    try {
        const response = await fetch(`/api/file?path=${encodeURIComponent(entry.path)}`, {
            method: "DELETE",
            cache: "no-store",
        });

        const data = await response.json().catch(() => ({}));
        if (!response.ok) {
            throw new Error(data.error || "Failed to delete file");
        }

        loadFiles(currentPath);
    } catch (error) {
        console.error(error.message || "Failed to delete file");
    }
}

function renderBreadcrumbs(path) {
    const container = document.getElementById("breadcrumbs");
    clearNode(container);

    let partial = "/";
    container.appendChild(makeButton("SD Card", "crumb", () => loadFiles("/")));

    path.split("/").filter(Boolean).forEach((segment) => {
        const sep = document.createElement("span");
        sep.className = "crumb-sep";
        sep.textContent = "/";
        container.appendChild(sep);

        partial = partial === "/" ? `/${segment}` : `${partial}/${segment}`;
        container.appendChild(makeButton(segment, "crumb", () => loadFiles(partial)));
    });
}

function renderEntries(path, entries) {
    const list = document.getElementById("file-list");
    clearNode(list);

    entries.sort((a, b) => {
        if (a.type !== b.type) {
            return a.type === "dir" ? -1 : 1;
        }
        return a.name.localeCompare(b.name);
    });

    if (path !== "/") {
        const upRow = document.createElement("li");
        upRow.className = "entry";

        const upButton = makeButton("..", "entry-link folder", () => loadFiles(getParentPath(path)));
        const upMeta = document.createElement("span");
        upMeta.className = "entry-meta";
        upMeta.textContent = "Up one level";

        const spacer = document.createElement("span");
        spacer.className = "entry-actions-spacer";

        upRow.appendChild(upButton);
        upRow.appendChild(upMeta);
        upRow.appendChild(spacer);
        list.appendChild(upRow);
    }

    if (!entries.length) {
        const empty = document.createElement("li");
        empty.className = "empty";
        empty.textContent = "This folder is empty.";
        list.appendChild(empty);
        return;
    }

    entries.forEach((entry) => {
        const row = document.createElement("li");
        row.className = "entry";

        if (entry.type === "dir") {
            const openButton = makeButton(entry.name, "entry-link folder", () => loadFiles(entry.path));
            const meta = document.createElement("span");
            meta.className = "entry-meta";
            meta.textContent = "Folder";

            const spacer = document.createElement("span");
            spacer.className = "entry-actions-spacer";

            row.appendChild(openButton);
            row.appendChild(meta);
            row.appendChild(spacer);
        } else {
            const name = document.createElement("span");
            name.className = "entry-name";
            name.textContent = entry.name;

            const meta = document.createElement("span");
            meta.className = "entry-meta";
            meta.textContent = formatSize(entry.size);

            const download = makeIconLink(
                "download",
                `/download?path=${encodeURIComponent(entry.path)}`,
                `Download ${entry.name}`,
                downloadIcon,
            );
            download.setAttribute("download", "");

            const remove = makeIconButton(
                "delete",
                `Delete ${entry.name}`,
                trashIcon,
                () => deleteFile(entry),
            );

            const actions = document.createElement("div");
            actions.className = "entry-actions";
            actions.appendChild(download);
            actions.appendChild(remove);

            row.appendChild(name);
            row.appendChild(meta);
            row.appendChild(actions);
        }

        list.appendChild(row);
    });
}

async function loadFiles(path) {
    try {
        const response = await fetch(`/api/files?path=${encodeURIComponent(path)}`, { cache: "no-store" });
        const data = await response.json();

        if (!response.ok) {
            throw new Error(data.error || "Failed to load files");
        }

        currentPath = data.path;
        renderBreadcrumbs(data.path);
        renderEntries(data.path, data.entries || []);
    } catch (error) {
        document.getElementById("breadcrumbs").textContent = "";
        clearNode(document.getElementById("file-list"));
        console.error(error.message || "Failed to load files");
    }
}

loadFiles("/");
