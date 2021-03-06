
// Called as each tab is activated
function openTab(evt, cityName) 
{
   var i, tabcontent, tablinks;
   tabcontent = document.getElementsByClassName("tabcontent");
   for (i = 0; i < tabcontent.length; i++) 
   {
      tabcontent[i].style.display = "none";
   }
   tablinks = document.getElementsByClassName("tablinks");
   for (i = 0; i < tablinks.length; i++) 
   {
      tablinks[i].className = tablinks[i].className.replace(" active", "");
   }
   document.getElementById(cityName).style.display = "block";
   evt.currentTarget.className += " active";
}

// Caled whenever the status indicator needs updating
function updateConnectionStatus(statusColour)
{ 
   var element = document.getElementById("connectionStatus");
   element.style.backgroundColor = statusColour; //"red";
}

// Called after form input is processed
function startConnect() {
   // Generate a random client ID
   clientID = "clientID-" + parseInt(Math.random() * 100);

   // Fetch the hostname/IP address and port number from the form
   host = document.getElementById("host").value;
   port = document.getElementById("port").value;

   // Print output for the user in the messages div
   document.getElementById("messages").innerHTML += '<span>Connecting to: ' + host + ' on port: ' + port + '</span><br/>';
   document.getElementById("messages").innerHTML += '<span>Using the following client value: ' + clientID + '</span><br/>';

   // Initialize new Paho client connection
   client = new Paho.MQTT.Client(host, Number(port), clientID);

   // Set callback handlers
   client.onConnectionLost = onConnectionLost;
   client.onMessageArrived = onMessageArrived;

   // Connect the client, if successful, call onConnect function
   client.connect({ 
       onSuccess: onConnect,
   });
}

// Called when the client connects
function onConnect() {
   // Fetch the MQTT topic from the form
   topic = document.getElementById("topic").value;

   // Print output for the user in the messages div
   document.getElementById("messages").innerHTML += '<span>Subscribing to: ' + topic + '</span><br/>';

   // Subscribe to the requested topic
   client.subscribe(topic);
   updateConnectionStatus('green');
}

// Called when the client loses its connection
function onConnectionLost(responseObject) {
   console.log("onConnectionLost: Connection Lost");
   if (responseObject.errorCode !== 0) {
       console.log("onConnectionLost: " + responseObject.errorMessage);
       updateConnectionStatus('red');
   }
}

// send a message 
function publish () {
  var myCmd = document.getElementById("cmd");
  var myRobot = document.getElementById("robotID");
  var mqttTopic = myRobot.value + "/commands";
  message = new Paho.MQTT.Message(myCmd.value);
  message.destinationName = mqttTopic;
  message.qos = 1;
  client.send(message);
}

// Called when a message arrives
function onMessageArrived(message) {
   console.log("onMessageArrived: " + message.payloadString);
   document.getElementById("messages").innerHTML += '<span>Topic: ' + message.destinationName + '  | ' + message.payloadString + '</span><br/>';
   updateScroll(); // Scroll to bottom of window
}

// Called when the disconnection button is pressed
function startDisconnect() {
   client.disconnect();
   updateConnectionStatus('red');
   document.getElementById("messages").innerHTML += '<span>Disconnected</span><br/>';
   updateScroll(); // Scroll to bottom of window
}

// Updates #messages div to auto-scroll
function updateScroll() {
   var element = document.getElementById("messages");
   element.scrollTop = element.scrollHeight;
}