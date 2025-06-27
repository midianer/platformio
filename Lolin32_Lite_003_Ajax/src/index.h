const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<body>

<div id="demo">
<h1>The LOLIN32 Lite Update web page without refresh</h1>
<h2>Lolin32_Lite_003_Ajax</h2>
	<button type="button" id="B1" onclick="sendData(1)">LED ON</button>
	<button type="button" id="B2" onclick="sendData(0)">LED OFF</button><BR>
	<button type="button" id="ID_DOWN" onclick="func_down()">Down</button>
	<button type="button" id="ID_UP" onclick="func_up()">Up</button>
	<button type="button" id="ID_DOWN_S" >Down</button><BR>
</div>

<div>
	ADC Value is : <span id="ADCValue">0</span><br>
    LED State is : <span id="LEDState">NA</span>
</div>
<script>
function func_down() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      //document.getElementById("ID_DOWN").innerHTML =
      //this.responseText;
    }
  };
  xhttp.open("GET", "Down", true);
  xhttp.send();
  setIntervalX(function () {
    getData();
    getButton();
  }, 1000, 15);
}


function func_up() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ID_UP").innerHTML =
      this.responseText;
    }
  };
  xhttp.open("GET", "Up", true);
  xhttp.send();
}


function sendData(led) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("LEDState").innerHTML =
      this.responseText;
    }
  };
  console.log("setLEDsetLEDsetLED");
  xhttp.open("GET", "setLED?LEDstate="+led, true);
  xhttp.send();
}

//setInterval(function() {
//  // Call a function repetatively with 2 Second interval
//  getData();
//  getButton();
//}, 2000);


function setIntervalX(callback, delay, repetitions) {
    var x = 0;
    var intervalID = window.setInterval(function () {

       callback();

       if (++x === repetitions) {
           window.clearInterval(intervalID);
       }
    }, delay);
}

// This will be repeated 5 times with 1 second intervals:
//setIntervalX(function () {
//  getData();
//  getButton();
//}, 1000, 15);

function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ADCValue").innerHTML =
      this.responseText;
      document.getElementById("ADCValue").style.backgroundColor = "lightblue";      
      document.getElementById("B1").style.backgroundColor = "lightblue";      
      document.getElementById("B2").style.backgroundColor = "green";
      console.log(this.responseText);
    }
  };
  xhttp.open("GET", "readADC", true);
  xhttp.send();
}

function getButton() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      console.log("DownState");
      document.getElementById("ID_DOWN_S").innerHTML = this.responseText;
      console.log(this.responseText);
    }
  };
  xhttp.open("GET", "DownState", true);
  xhttp.send();
}



</script>
<br><br><a href="https://circuits4you.com/2018/02/04/esp8266-ajax-update-part-of-web-page-without-refreshing/">Circuits4you.com modified</a>
</body>
</html>
)=====";
