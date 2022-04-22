const express = require('express')
var udp = require('dgram');
 var server = udp.createSocket("udp4");

server.on("/trash", function (msg) {
    process.stdout.write("UDP String: " + msg + "\n");
    process.exit();
})
    .bind(2205, () => {
        server.setSendBufferSize(12345);
        const size = server.getSendBufferSize();
        console.log(size);

    });

// Client sending message to server
client.send("Hello", 0, 7, 2205, "localhost");

app = express(),
port = process.env.PORT || 3000;

app.listen(port);
app.get('/', (req, res) => res.send('Hello World!'));

app.get('/trash', (req, res) => res.send('data'));

console.log('book list RESTful API server started on: ' + port);