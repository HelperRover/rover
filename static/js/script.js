// This function is called from index.html and the parameter
// changes depending on which div is clicked.
function robotMove(direction) {

    // Declare a variable to hold the route.
    var url;

    // Switch statement to select route based on the div
    // that was clicked.
    switch (direction) {
        case 1:
            url = "/forward";
            break;
        case 2:
            url = "/left";
            break;
        case 3:
            url = "/right";
            break;
        case 4:
            url = "/backward";
            break;
        case 5:
            url = "/automatic"
            break;
        case 6:
            url = "/stop"
            break;
    }

    // Make an AJAX call to the server with the 
    // route. The server will respond by executing
    // the respective code in the app.py file.
    if (url) {
        var oReq = new XMLHttpRequest();
        oReq.open("GET", url);
        oReq.send();
    }
}

// This function follows the same format as the function above
// but instead of mouse clicks it follows keyboard events. This 
// allows for the keyboard arrows to control the rover. Also,
// clicking "a" engages automatic control.
function keyEvent(e) {
    var url;

    switch(e.key) {
		case "ArrowLeft":
		case "Left":
			url = "/left";
			break;

		case "ArrowRight":
		case "Right":
			url = "/right";
			break;

		case "ArrowUp":
		case "Up":
			url = "/forward";
			break;

		case "ArrowDown":
		case "Down":
			url = "/backward";
			break;
            
        case "a":
            url = "/automatic"
            break;
    }

    if (url) {
        var oReq = new XMLHttpRequest();
        oReq.open("GET", url);
        oReq.send();
    }
}

// This event listener is for the keyEvent() function above.
// It listens for key presses and then calls the keyEvent function.
document.addEventListener("keydown", keyEvent, false);