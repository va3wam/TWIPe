
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

// Called whenever the status indicator needs updating
function updateConnectionStatus(statusColour, statusText)
{ 
   var element = document.getElementById("connectionStatus");
   element.style.backgroundColor = statusColour; //"red";
   element.value = statusText;
}

// Called after form input is processed
function startConnect() 
{
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
function onConnect() 
{
   // Fetch the MQTT topic from the form
   topic = document.getElementById("topic").value;

   // Print output for the user in the messages div
   document.getElementById("messages").innerHTML += '<span>Subscribing to: ' + topic + '</span><br/>';

   // Subscribe to the requested topic
   client.subscribe(topic);
   updateConnectionStatus('green','Connected');
}

// Called when the client loses its connection
function onConnectionLost(responseObject) 
{
   console.log("onConnectionLost: Connection Lost");
   if (responseObject.errorCode !== 0) 
   {
       console.log("onConnectionLost: " + responseObject.errorMessage);
       updateConnectionStatus('red','Disconnected');
   }
}

// Send a message 
function publish(parm) 
{
   switch(parm) 
   {
      case 'iGain':
         var myCmd = document.getElementById("iGain");
         var myRobot = document.getElementById("robotID");
         var mqttTopic = myRobot.value + "/commands";
         message = new Paho.MQTT.Message("setvar,balance.pidIGain," + myCmd.value);
         message.destinationName = mqttTopic;
         message.qos = 1;
         client.send(message);
         break;
      case 'iCount':
         var myCmd = document.getElementById("iCount");
         var myRobot = document.getElementById("robotID");
         var mqttTopic = myRobot.value + "/commands";
         message = new Paho.MQTT.Message("setvar,balance.pidICount," + myCmd.value);
         message.destinationName = mqttTopic;
         message.qos = 1;
         client.send(message);
         break;
      case 'pGain':
         var myCmd = document.getElementById("pGain");
         var myRobot = document.getElementById("robotID");
         var mqttTopic = myRobot.value + "/commands";
         message = new Paho.MQTT.Message("setvar,balance.pidPGain," + myCmd.value);
         message.destinationName = mqttTopic;
         message.qos = 1;
         client.send(message);
         break;
      case 'dGain':
         var myCmd = document.getElementById("dGain");
         var myRobot = document.getElementById("robotID");
         var mqttTopic = myRobot.value + "/commands";
         message = new Paho.MQTT.Message("setvar,balance.pidDGain," + myCmd.value);
         message.destinationName = mqttTopic;
         message.qos = 1;
         client.send(message);
         break;      
      case 'slowTicks':
         var myCmd = document.getElementById("slowTicks");
         var myRobot = document.getElementById("robotID");
         var mqttTopic = myRobot.value + "/commands";
         message = new Paho.MQTT.Message("setvar,balance.slowTicks," +  myCmd.value);
         message.destinationName = mqttTopic;
         message.qos = 1;
         client.send(message);
         break;
      case 'fastTicks':
         var myCmd = document.getElementById("fastTicks");
         var myRobot = document.getElementById("robotID");
         var mqttTopic = myRobot.value + "/commands";
         message = new Paho.MQTT.Message("setvar,balance.fastTicks," +  myCmd.value);
         message.destinationName = mqttTopic;
         message.qos = 1;
         client.send(message);
         break;
      case 'smoother':
            var myCmd = document.getElementById("smoother");
            var myRobot = document.getElementById("robotID");
            var mqttTopic = myRobot.value + "/commands";
            message = new Paho.MQTT.Message("setvar,balance.smoother," +  myCmd.value);
            message.destinationName = mqttTopic;
            message.qos = 1;
            client.send(message);
            break;
      case 'targetAngle':
         var myCmd = document.getElementById("targetAngle");
         var myRobot = document.getElementById("robotID");
         var mqttTopic = myRobot.value + "/commands";
         message = new Paho.MQTT.Message("setvar,balance.targetAngle," +  myCmd.value);
         message.destinationName = mqttTopic;
         message.qos = 1;
         client.send(message);
         break;      
      case 'activeAngle':
         var myCmd = document.getElementById("activeAngle");
         var myRobot = document.getElementById("robotID");
         var mqttTopic = myRobot.value + "/commands";
         message = new Paho.MQTT.Message("setvar,balance.activeAngle," +  myCmd.value);
         message.destinationName = mqttTopic;
         message.qos = 1;
         client.send(message);
         break;         
      case 'tmrImu':
         var myCmd = document.getElementById("tmrImu");
         var myRobot = document.getElementById("robotID");
         var mqttTopic = myRobot.value + "/commands";
         message = new Paho.MQTT.Message("setvar,balance.tmrImu," +  myCmd.value);
         message.destinationName = mqttTopic;
         message.qos = 1;
         client.send(message);
         break;            
      case 'balTel':
         var myCmd = document.getElementsByName("balTel");
         var myRobot = document.getElementById("robotID");
         var mqttTopic = myRobot.value + "/commands";
         for(i = 0; i < myCmd.length; i++) 
         { 
            if(myCmd[i].checked) 
            {
               message = new Paho.MQTT.Message("setvar," + myCmd[i].value);
            } //if
         } //for 
         message.destinationName = mqttTopic;      
         message.qos = 1;
         client.send(message);
         break;               
      case 'healthTel':
         var myCmd = document.getElementsByName("healthTel");
         var myRobot = document.getElementById("robotID");
         var mqttTopic = myRobot.value + "/commands";
         for(i = 0; i < myCmd.length; i++) 
         { 
            if(myCmd[i].checked) 
            {
               message = new Paho.MQTT.Message("setvar," + myCmd[i].value);
            } //if
         } //for 
         message.destinationName = mqttTopic;      
         message.qos = 1;
         client.send(message);
         break;                  
      case 'custom':
         var myCmd = document.getElementById("cmd");
         var myRobot = document.getElementById("robotID");
         var mqttTopic = myRobot.value + "/commands";
         message = new Paho.MQTT.Message(myCmd.value);
         message.destinationName = mqttTopic;
         message.qos = 1;
         client.send(message);
         break;   
      default:
         window.alert("Topic tree " + parm + " unknown. Fix HTML and/or Javascript");
         break;
    }   
}

// Called when a message arrives
function onMessageArrived(message) 
{
   document.getElementById("messages").innerHTML += '<span>Topic: ' + message.destinationName + '/' + message.payloadString + '</span><br/>'; // Put raw incoming message into the session log
   updateScroll(); // Scroll to bottom of window
   var topicElements = message.destinationName.split("/"); // Parse into pieces using forward slash to split message
   var cmdElements = message.payloadString.split(","); // Parse into pieces using comma to split message 
   switch(topicElements[1]) 
   {
      case 'commands':
         break;
      case 'hthTel':
         document.getElementById("messages").innerHTML += '<span>Set wifiCon to ' + cmdElements[1] + '</span><br/>';
         updateScroll(); // Scroll to bottom of window
         document.getElementById("wifiCon").value = cmdElements[1];
         document.getElementById("wifiDrop").value = cmdElements[2];
         document.getElementById("mqttCon").value = cmdElements[3];
         document.getElementById("mqttDrop").value = cmdElements[4];
         document.getElementById("dmpMissing").value = cmdElements[5];
         document.getElementById("unknownCmd").value = cmdElements[6];
         document.getElementById("leftDrvFault").value = cmdElements[7];
         document.getElementById("rightDrvFault").value = cmdElements[8];
         break;   
      case 'balTel':
         var canvas = document.getElementById("myCanvas");
         var ctx2 = canvas.getContext("2d");
         ctx2.fillStyle='#333';
         ctx2.fillRect(50,50,100,100);
         var ctx = canvas.getContext("2d");
         ctx.fillStyle='red';
         var deg = Math.PI/180;
         ctx.save();
         ctx.translate(100, 100);
         ctx.rotate(45 * deg);
         ctx.fillRect(-50,-50,100,100);
         ctx.restore();
         break;
      default:
         document.getElementById("messages").innerHTML += '<span> The topic ' + topicElements[1] + ' is unknown. Check onMessageArrived() in mainApp.js.</span><br/>';
         updateScroll(); // Scroll to bottom of window
         document.getElementById("messages").innerHTML += '<span>Number of pieces of command is ' + cmdElements.length + '</span><br/>';
         updateScroll(); // Scroll to bottom of window
         document.getElementById("messages").innerHTML += '<span>Part 1 is ' + cmdElements[0] + '</span><br/>';
         updateScroll(); // Scroll to bottom of window
         document.getElementById("messages").innerHTML += '<span>Part 2 is ' + cmdElements[1] + '</span><br/>';
         updateScroll(); // Scroll to bottom of window
         document.getElementById("messages").innerHTML += '<span>Part 3 is ' + cmdElements[2] + '</span><br/>';
         updateScroll(); // Scroll to bottom of window
break;
   } //switch   
} //onMessageArrived()

// Called when the disconnection button is pressed
function startDisconnect() 
{
   client.disconnect();
   updateConnectionStatus('red','Disconnected');
   document.getElementById("messages").innerHTML += '<span>Disconnected</span><br/>';
   updateScroll(); // Scroll to bottom of window
}

// Updates #messages div to auto-scroll
function updateScroll() 
{
   var element = document.getElementById("messages");
   element.scrollTop = element.scrollHeight;
}