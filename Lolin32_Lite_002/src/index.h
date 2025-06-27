// HTML web page
//const char index_html[] PROGMEM = R"rawliteral(
const char index_html[] = R"rawliteral(
<!DOCTYPE HTML><html>
  <head>
    <title>ESP Rolladen Taster</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial; text-align: center; margin:0px auto; padding-top: 30px;}
      .button {
        padding: 10px 20px;
        font-size: 24px;
        text-align: center;
        outline: none;
        color: #fff;
        background-color: #2f4468;
        border: none;
        border-radius: 5px;
        box-shadow: 0 6px #999;
        cursor: pointer;
        -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
        user-select: none;
        -webkit-tap-highlight-color: rgba(0,0,0,0);
      }
      .button:hover {background-color: #cf2e45}
      .button:active {
        background-color: #0000ff;
        <!--box-shadow: 0 4px #666;    -->
        <!--transform: translateY(2px);-->
      }
      .test{
         height:100px;
         width:100px;
      }
    </style>
  </head>
  <body>
    <h1>Rolladen Taster Maja</h1>
    <h2>rolladen_taster.ino</h2>
    <div style="border: 10px solid white">
       <button class="button" id="mousedown" onmousedown="toggleCheckbox('auf');" ontouchstart="toggleCheckbox('auf');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">AUF</button>
       <button1 class="button" onmousedown="toggleCheckbox('ab');" ontouchstart="toggleCheckbox('ab');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">AB</button1>
    </div>
    <div>
      <button  id="votebutton">Submit</button>
      <button id="votebutton1" type="button">Submit</button>
    </div>
    <div>
        <button class="test">Start</button>
        <button class="test">Stop</button>
    </div>
    <div style="border: 10px solid white">
       <button class="button" onclick="SendDauerTaste('aufD');">AUF_D</button>
       <button class="button" onclick="SendDauerTaste('abD');">AB_D</button>
       <button class="button" onclick="SendDauerTaste('stopD');">STOP</button>
    </div>
   <script>
   function toggleCheckbox(x) {
     var xhr = new XMLHttpRequest();
     xhr.onreadystatechange = function() {
     if (this.readyState == 4 && this.status == 200) {
         document.getElementById("mousedown").innerHTML =
      this.responseText;
      document.getElementById("mousedown").style.backgroundColor = "#8ccdef";      
      document.getElementById("mousedown").textContent = "MusicStartStopDn\n";
      console.log(this.responseText);
      console.log(document.getElementById("mousedown").textContent);
    }
  };
     xhr.open("GET", "/" + x, true);
     xhr.send();
   }
   function SendDauerTaste(x) {
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/dauer?dauersend=" + x, true);
     xhr.send();
   }
   function submitPoll(){
     document.getElementById("votebutton").disabled = true;
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/hallo", true);
     xhr.send();
     setTimeout(function() {
       document.getElementById("votebutton").disabled = false;
     }, 5000);
   }
   function submitPoll1(x){
     document.getElementById("votebutton1").disabled = true;
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/" + x, true);
     xhr.send();
     setTimeout(function() {
       document.getElementById("votebutton1").disabled = false;
     }, 8000);
   }
   document.getElementById("votebutton").addEventListener("click", submitPoll);  
   document.getElementById("votebutton").style.fontSize = "25px";
   document.getElementById("votebutton1").style.fontSize = "25px";
   document.getElementById("votebutton1").addEventListener("click", function() {
        submitPoll1("abneu"); });
    </script>
  </body>
</html>)rawliteral";
