document.addEventListener("DOMContentLoaded", function () {
    const blocks = document.querySelectorAll(".admonition");
    blocks.forEach((block) => {
        const title = block.querySelector(".admonition-title");
        if (title && title.innerText.trim() === "Question") {
            // Apply styles to the entire Question block
            block.style.backgroundColor = "#e6ccff"; // Pastel purple background for the block
            block.style.borderLeft = "0px solid #e6ccff"; // Pastel purple border for emphasis

            // Apply styles to the title bar of the Question block
            title.style.backgroundColor = "#9933ff"; // Dark purple for the title bar
            title.style.color = "#ffffff"; // White text for the title
            title.style.fontWeight = "bold"; // Make title text bold
            title.style.padding = "5px 10px"; // Add padding inside the title
            title.style.borderBottom = "0px solid #e6ccff"; // Subtle separator below the title
        }
    });
});


document.addEventListener("DOMContentLoaded", function () {
    const blocks = document.querySelectorAll(".admonition");
    blocks.forEach((block) => {
        const title = block.querySelector(".admonition-title");
        if (title && title.innerText.trim().startsWith("Thymio")) {
            // Apply styles to the entire Thymio block
            block.style.backgroundColor = "#ddefff"; // Soft dark blue background for the block
            block.style.borderLeft = "0px solid #99ccff"; // Light blue border for emphasis

            // Apply styles to the title bar of the Thymio block
            title.style.backgroundColor = "#003366"; // Dark blue for the title bar
            title.style.color = "#ffffff"; // White text for the title
            title.style.fontWeight = "bold"; // Make title text bold
            title.style.padding = "5px 10px"; // Add padding inside the title
            title.style.borderBottom = "0px solid #99ccff"; // Subtle separator below the title
        }
    });
});

document.addEventListener("DOMContentLoaded", function () {
    const blocks = document.querySelectorAll(".admonition");
    blocks.forEach((block) => {
        const title = block.querySelector(".admonition-title");
        if (title && title.innerText.trim() === "Hints") {
            // Apply styles to the entire Hints block
            block.style.backgroundColor = "#e0f4ff"; // Lighter blue background for the block
            block.style.borderLeft = "0px solid #66a3ff"; // Soft blue border for emphasis

            // Apply styles to the title bar of the Hints block
            title.style.backgroundColor = "#005b99"; // Vibrant blue for the title bar
            title.style.color = "#ffffff"; // White text for the title
            title.style.fontWeight = "bold"; // Make title text bold
            title.style.padding = "5px 10px"; // Add padding inside the title
            title.style.borderBottom = "0px solid #66a3ff"; // Subtle separator below the title
        }
    });
});


document.addEventListener("DOMContentLoaded", function () {
    const blocks = document.querySelectorAll(".admonition");
    blocks.forEach((block) => {
        const title = block.querySelector(".admonition-title");
        if (title && title.innerText.trim().startsWith("Task")) {
            // Apply styles to the entire Thymio block
            block.style.backgroundColor = "#ddefff"; // Soft dark blue background for the block
            block.style.borderLeft = "0px solid #99ccff"; // Light blue border for emphasis

            // Apply styles to the title bar of the Thymio block
            title.style.backgroundColor = "#003366"; // Dark blue for the title bar
            title.style.color = "#ffffff"; // White text for the title
            title.style.fontWeight = "bold"; // Make title text bold
            title.style.padding = "5px 10px"; // Add padding inside the title
            title.style.borderBottom = "0px solid #99ccff"; // Subtle separator below the title
        }
    });
});

