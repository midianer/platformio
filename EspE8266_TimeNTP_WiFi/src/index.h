const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<head>
  <title>EspE8266_TimeNTP_WiFi</title>
  <style>
    .button {
      border: none;
      color: black;
      background: #ff0000;
      padding: 15px 32px;
      width: 800px;
      text-align: center;
      text-decoration: none;
      display: inline-block;
      font-size: 24px;
      margin: 4px 2px;
      cursor: pointer;
    }
  </style>
</head>
<html>
<body>

<div id="demo">
<h1>EspE8266_TimeNTP_WiFi</h1>
<h2>Guenter Kern Ingbuero</h2>
</div>

<div>
<button class="button" id="IDDate" onclick="getDate()" >GetDate</button>
</div>
<div>
<button class="button" id="IDRSSI" onclick="getRSSI()" >GetRSSI</button>
</div>

<script>

setInterval(function() {
  // Call a function repetatively with 2 Second interval
  getDate();
  getRSSI();
}, 1000);

function getDate() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("IDDate").innerHTML = "Date/Time = " + this.responseText;
      console.log(this.responseText);
    }
  };
  xhttp.open("GET", "GetDate", true);
  xhttp.send();
}

function getRSSI() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("IDRSSI").innerHTML = "RSSI = " + this.responseText;
      console.log(this.responseText);
    }
  };
  xhttp.open("GET", "GetRSSI", true);
  xhttp.send();
}

</script>

</body>
</html>
)=====";
