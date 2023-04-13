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
            url = "/stop"
            break;
        case 6:
            url = "/automatic"
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