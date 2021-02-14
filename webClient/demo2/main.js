// client, user and device details
var serverUrl   = "192.168.2.21";     /* wss://mqtt.cumulocity.com/mqtt for a secure connection */
var clientId    = "my_mqtt_js_client";
var device_name = "My JS MQTT device";
var tenant      = "<<tenant_ID>>";
var username    = "<<username>>";
var password    = "<<password>>";

var undeliveredMessages = [];
var temperature = 25;

// display all messages on the page
function log (message) {
    document.getElementById("logger").insertAdjacentHTML("beforeend", "<div>" + message + "</div>");
}

// configure the client to Cumulocity
var client = new Paho.MQTT.Client(serverUrl, clientId);

// send a message 
function publish (topic, message, onMessageDeliveredCallback) {
    message = new Paho.MQTT.Message(message);
    message.destinationName = topic;
    message.qos = 2;
    undeliveredMessages.push({
        message: message,
        onMessageDeliveredCallback: onMessageDeliveredCallback
    });
    client.send(message);
}

// display all incoming messages
client.onMessageArrived = function (message) {
    log('Received operation "' + message.payloadString + '"');
    if (message.payloadString.indexOf("510") === 0) {
        log("Simulating device restart...");
        publish("s/us", "501,c8y_Restart");
        log("...restarting...");
        setTimeout(function() {
            publish("s/us", "503,c8y_Restart");
        }, 1000);
        log("...done...");
    }
};

// display all delivered messages
client.onMessageDelivered = function onMessageDelivered (message) {
    log('Message "' + message.payloadString + '" delivered');
    var undeliveredMessage = undeliveredMessages.pop();
    if (undeliveredMessage.onMessageDeliveredCallback) {
        undeliveredMessage.onMessageDeliveredCallback();
    }
};

function createDevice () {
    // register a new device
    publish("s/us", "100," + device_name + ",c8y_MQTTDevice", function() {
        // set hardware information
        publish("s/us", "110,S123456789,MQTT test model,Rev0.1", function() {
            publish("s/us", "114,c8y_Restart", function() { 
                log("Enable restart operation support");
                // listen for operations
                client.subscribe("s/ds");
            });

            // send temperature measurement
            setInterval(function() {
                publish("s/us", "211," + temperature);
                temperature += 0.5 - Math.random();
            }, 3000);
        });
    });
}

// connect the client to Cumulocity
function init () {
    client.connect({
        userName: tenant + "/" + username,
        password: password,
        onSuccess: createDevice
    });
}

init();