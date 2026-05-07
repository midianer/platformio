const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<head>
  <title>S2Mini NTP</title>
  <style>
  </style>
</head>
<html>
<body>

<div id="demo">
<h1>S2Mini NTP</h1>
<h2>Guenter Kern Ingbuero</h2>
</div>

<script>

setInterval(function() {
  // Call a function repetatively with 2 Second interval
  getDate();
}, 1000);

function getDate() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      console.log(this.responseText);
    }
  };
  xhttp.open("GET", "GetDate", true);
  xhttp.send();
}

</script>

</body>
</html>
)=====";
