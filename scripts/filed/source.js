var field = document.getElementById("field").getContext("2d");

var field_container = document.getElementById("field_container");

let field_x = 0;
let field_y = 0;

field_container.onmousedown = function(e){
    field_x = e.pageX - 10;
    field_y = e.pageY - 10;
}

document.onkeydown = function(e) {
    if (e.key == "s") {
        field_y += 1;
    } else if (e.key == "w") {
        field_y -= 1;
    } else if (e.key == "a") {
        field_x -= 1;
    } else if (e.key == "d") {
        field_x += 1;
    } else {
        return;
    }

    update_robot_position();
}

function meters_to_pixels(m) {
    return m * 3.6576  / 12 * 600;
}

function pixels_to_meters(pixels) {
    return pixels / 3.6576 * 12 / 600;
}

function update_robot_position() {
    field.lineWidth = 2;
    field.clearRect(0, 0, 600, 600);

    field.beginPath();
    field.rect(field_x - meters_to_pixels(0.205), field_y - meters_to_pixels(0.2125), meters_to_pixels(0.205 * 2), meters_to_pixels(0.2125 * 2));
    field.closePath();
    field.stroke();

    field.beginPath();
    field.rect(field_x - meters_to_pixels(0.1835), field_y - meters_to_pixels(0.31), meters_to_pixels(0.1835 * 2), meters_to_pixels(0.31 - 0.26));
    field.closePath();
    field.stroke();

    console.log("click")
    document.getElementById("x").innerHTML = pixels_to_meters(-field_y + 300);
    document.getElementById("y").innerHTML = pixels_to_meters(field_x - 300);
}

field.strokeStyle = "#9acd32";