#include <WebServer.h>

#ifdef webServer

const char GYRO_PID[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<style>
form,input,h2
{
  color: black;
  text-align: center;
  font-size:6vw;
  font-family: monospace;
}
p
{
  text-align: center;
  font-size:6vw;
}

body
{
  background-color: lightblue;
}
</style>
</head>
<body>
<h2>EspCopter Gyro PID</h2>
<form action="/action_page">
  Gyro_P:  <input type="number" name="pid_p" value="%d" max="200" min="0">
  <br>
  Gyro_I:  <input type="number" name="pid_i" value="%d" max="200" min="0">
  <br>
  Gyro_D:  <input type="number" name="pid_d" value="%d" max="200" min="0">
  <br><br>
  <input type="submit" value="Submit">
</form> 
<p>
<a href="/gyro_page">Gyro PID</a>
<a href="/level_page">Level PID</a>
</p>
</body>
</html>
)=====";

const char ACC_PID[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<style>
form,input,h2
{
  color: black;
  text-align: center;
  font-size:6vw;
  font-family: monospace;
}
p
{
  text-align: center;
  font-size:6vw;
}
body
{
  background-color: lightblue;
}
</style>
</head>
<body>
<h2>EspCopter Level PID</h2>
<form action="/action_level">
  Level_P:  <input type="number" name="lev_p" value="%d" max="200" min="0">
  <br>
  Level_I:  <input type="number" name="lev_i" value="%d" max="200" min="0">
  <br>
  Level_D:  <input type="number" name="lev_d" value="%d" max="200" min="0">
  <br><br>
  <input type="submit" value="Submit">
</form> 
<p>
<a href="/gyro_page">Gyro PID</a>
<a href="/level_page">Level PID</a>
</p>
</body>
</html>
)=====";

//SSID and Password of your WiFi router
const char* ssid = "FSM";
const char* password = "0101010101";

WebServer server(80); //Server on port 80

//===============================================================
// This routine is executed when you open its IP in browser
//===============================================================
void handleRoot() 
{
 char temp[1000];
 snprintf(temp,1000,GYRO_PID, int(P_PID*100.0+0.4),int(I_PID*100.0+0.4),int(D_PID*100.0+0.4));
 server.send(200, "text/html", temp); //Send web page
 Serial.println("root page served");
}

void handleGyro() 
{
 char temp[1000];
 snprintf(temp,1000,GYRO_PID, int(P_PID*100.0+0.4),int(I_PID*100.0+0.4),int(D_PID*100.0+0.4));
 server.send(200, "text/html", temp); //Send web page
 Serial.println("gyro page served");
}

void handleLevel() 
{
 char temp[1000];
 snprintf(temp,1000,ACC_PID, int(P_Level_PID*100.0+0.4),int(I_Level_PID*100.0+0.4),int(D_Level_PID*100.0+0.4));
 server.send(200, "text/html", temp); //Send web page
 Serial.println("level page served");
}

//===============================================================
// This routine is executed when you press submit
//===============================================================

void action_pid() 
{
 int p = server.arg("pid_p").toInt(); 
 int i = server.arg("pid_i").toInt(); 
 int d = server.arg("pid_d").toInt(); 

 P_PID = float(p) * 0.01 + 0.004;
 I_PID = float(i) * 0.01 + 0.004;
 D_PID = float(d) * 0.01 + 0.004;

 PID_Store();
 Serial.println("Gyro pid stored");
 handleGyro();
}

void action_level() 
{
 int pl = server.arg("lev_p").toInt(); 
 int il = server.arg("lev_i").toInt(); 
 int dl = server.arg("lev_d").toInt(); 

 P_Level_PID = float(pl) * 0.01 + 0.004;
 I_Level_PID = float(il) * 0.01 + 0.004;
 D_Level_PID = float(dl) * 0.01 + 0.004;

 PID_Store();
 Serial.println("Level pid stored");
 handleLevel();
}

//==============================================================
//                  SETUP
//==============================================================
void setupwebserver(void)
{  
  WiFi.begin(ssid, password);     //Connect to your WiFi router
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }

  //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println("WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //IP address assigned to your ESP
 
  server.on("/", handleRoot);               //Which routine to handle at root location
  server.on("/gyro_page", handleGyro);      //Which routine to handle at gyro location
  server.on("/level_page", handleLevel);    //Which routine to handle at level location
  server.on("/action_page", action_pid);    //form action is handled here
  server.on("/action_level", action_level); //form action is handled here

  server.begin();                  //Start server
  Serial.println("HTTP server started");
}

//==============================================================
//                     LOOP
//==============================================================
void loopwebserver(void) { server.handleClient(); }
void stopwebserver(void) { server.stop(); }

#endif
