#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#define MAX_SRV_CLIENTS 3

MDNSResponder mdns;

const char* ssid     = "Rubicon Precision systems Wi-Fi";
const char* password = "newmembrane";
char message[255]="no connection";
String MESS;
String mess;
const char webPage[] PROGMEM = R"=====(<!DOCTYPE html>
<html>
   <style>
      body { background-color: #d7ecf4; font-family: Arial, Helvetica, Sans-Serif; Color: #2a4a8b; }\
    </style>
   <style type="text/css">
textarea {
 width: 300px;
 height: 40px;
 background-color: white;
 font-size: 1em;
 font-weight: bold;
 font-family: Verdana, Arial, Helvetica, sans-serif;
 border: 1px solid black;
}
.button {
  background-color: #e3e3e3;
  border: none;
  color: black;
  padding: 15px 32px;
  text-align: center;
  text-decoration: none;
  display: inline-block;
  font-size: 16px;
  margin: 4px 2px;
  cursor: pointer;
  line-height:50px;
  border-radius: 30px;
  vertical-align: top;
  border:solid black 1px;
  line-height:13px;
  font-size: 13px;
    
}
.button2 
{
  background-color: #e3e3e3;
  border: none;

  color: black;
  padding: 15px 32px;
  text-align: center;
  text-decoration: none;
  font-size: 16px;
  margin: 4px 2px;
  cursor: pointer;
  line-height:10px;
  border-radius: 10px;
  vertical-align: top;
  border:solid black 1px;
  line-height:13px;
  font-size: 13px;
    
}

.klapan_but
{
 
  border: none;
  color: black;
  padding: 15px 32px;
  text-align: center;
  text-decoration: none;
  font-size: 16px;
  margin: 4px 2px;
  cursor: pointer;
  line-height:10px;
  border-radius: 10px;
  vertical-align: top;
  border:solid black 1px;
  line-height:13px;
  font-size: 13px;
    
}

  </style>
  <center>
<div id="demo">

<h1>Rubicon Precision Systems</h1>

</div>
 
<div>
Data : <span id="Data">0</span><br>
</div>
    <TEXTAREA type="number" id="thetext" rows="1" cols="80" >Pressure in Pa</TEXTAREA>
    
  </p>
 
   <input  class="button"  type="button" id="Gas1"  onclick="gases(this)" value="Gas1">

   <input class="button"  type="button" id="Gas2"  onclick="gases(this)" value="Gas2">

   <input class="button" type="button" id="Gas3"  onclick="gases(this)" value="Gas3">
<p> 
   <input id="sub" class="button2" type="button" name="Submit" value="Start_process" onclick="myFunction()" /> 
 <input id="reset" class="button2" type="reset"  value="Reset"  onclick="broke_process()" /> 

</p>
<input class="klapan_but" type="button" id="klapan_4"  onmousedown="myFunctionButton(this)" value="4" />
<input class="klapan_but" type="button" id="klapan_5"  onmousedown="myFunctionButton(this)" value="5" />
<input class="klapan_but" type="button" id="klapan_6"  onmousedown="myFunctionButton(this)" value="6" />
<input class="klapan_but" type="button" id="klapan_9"  onmousedown="myFunctionButton(this)" value="9" />
<input class="klapan_but" type="button" id="klapan_10"  onmousedown="myFunctionButton(this)" value="10" />
<input class="klapan_but" type="button" id="klapan_11"  onmousedown="myFunctionButton(this)" value="11" />
<input class="klapan_but" type="button" id="klapan_13"  onmousedown="myFunctionButton(this)" value="13" />
<p>stage: <input  type="button" id="next_state"  onclick="value_change(this)"  value="0"/> </p>
<input type="button" id="zero_all_rel" onclick="zero_all()" value="RESET ALL"/>
<script>
   var num = document.getElementById('thetext').value;
  document.getElementById('thetext').addEventListener('keyup', function (e){
  var key = window.event.keyCode;

    // If the user has pressed enter
    if (key === 13) 
    {
       myFunction();
        return false;
    }
    else if(key==8)
    {
      num = document.getElementById('thetext').value;
      
      return false;
    }
    else if((key<48)||(key>58))
    {
      
      this.value=num;
      
      return false;

    }
    else
     {
      num = document.getElementById('thetext').value;
        return true;
     }
}, false);
  var gas1 =document.getElementById("Gas1");
  var gas2 =document.getElementById("Gas2");
  var gas3 =document.getElementById("Gas3");
  var area=document.getElementById("thetext");
  var s_b = document.getElementById("sub");
  var n_s = document.getElementById("next_state");
  var data_string="";
  var but_style=cument.getElementById("Gas3").style.backgroundColor;
function myFunction() 
{
 var xhttp = new XMLHttpRequest();
 data_string=':';
  if(isNaN(area.value))
  {
    alert("bad entery");
    area.style.backgroundColor='red';
    area.value="";
  }
  else
  {
    
    if((area.value>303975))
    {
      alert("need pressure under 303975Pa");
      area.style.backgroundColor='red';
    }
    if(area.value<1000)
    {
      alert("need pressure more than 1000Pa");
      area.style.backgroundColor='red';
    }
    else
    {
      if(click_ok)
      {
        alert("Start process");
        
        area.disabled=true;
        gas1.disabled=true;
        gas2.disabled=true;
        gas3.disabled=true;
        area.style.backgroundColor='gray';
        s_b.disabled=true;    
        s_b.style.backgroundColor='gray';
      if(gas1.style.backgroundColor=='green')
        {
        data_string+='1';
        gas2.style.backgroundColor='gray';
        gas3.style.backgroundColor='gray';
        }
      if(gas2.style.backgroundColor=='green')
        {
          data_string+='2';
        gas1.style.backgroundColor='gray';
        gas3.style.backgroundColor='gray';
        }
      if(gas3.style.backgroundColor=='green')
        {
          data_string+='3';
        gas2.style.backgroundColor='gray';
        gas1.style.backgroundColor='gray';
        }
        data_string+=':';
        data_string+=area.value;
        data_string+=':';
        data_string+='1';
        data_string+='\n';
      // alert(data_string);
        xhttp.open("GET","data_string?data_string="+data_string,true);
        xhttp.send();
        data_string="";
       v
      }
    
    else
      {

        alert("Choose Gas");
      }
    }
    
  }
    
}
var click_ok=false;
function gases(elem)
{

  
  click_ok=true;
    elem.style.backgroundColor='green';
    if(elem.id=="Gas1")
    {
      gas2.style.backgroundColor='lightgray';
      gas3.style.backgroundColor='lightgray';
    }
    if(elem.id=="Gas2")
    {
      gas1.style.backgroundColor='lightgray';
      gas3.style.backgroundColor='lightgray';
    }
    if(elem.id=="Gas3")
    {
      gas2.style.backgroundColor='lightgray';
      gas1.style.backgroundColor='lightgray';
    }
}

function broke_process()
{
var xhttp = new XMLHttpRequest();
area.disabled=false;
s_b.disabled=false;
gas1.disabled=false;
gas2.disabled=false;
gas3.disabled=false;
area.style.backgroundColor='white';
gas2.style.backgroundColor='#e3e3e3';
gas1.style.backgroundColor='#e3e3e3';
gas3.style.backgroundColor='#e3e3e3';
s_b.style.backgroundColor='#e3e3e3';
click_ok=false;
data_string=':0:0000:0\n';
n_s.value=0;
//alert(data_string);

xhttp.open("GET","data_string?data_string="+data_string,true);
xhttp.send();
data_string="";

}


function myFunctionButton(elmnt) {
  

  var xhttp = new XMLHttpRequest();
  if(elmnt.value=="10")
    {data_string ='1';}
  else if(elmnt.value=="11")
    {data_string ='2';}
  else if(elmnt.value=="13")
    {data_string ='3';}
  else{data_string = elmnt.value;}
   if(elmnt.style.backgroundColor == 'green')
    {
    elmnt.style.backgroundColor = 'red';
    data_string+=1;
    
    }
    else
    {
    elmnt.style.backgroundColor = 'green';
    data_string+=0;
    }
  data_string+=":0000:0\n";
  //alert(data_string);
  xhttp.open("GET","data_string?data_string="+data_string,true);
  xhttp.send();
  data_string="";
}

function value_change(elemnt)
{
   var xhttp = new XMLHttpRequest();
   elemnt.value++;
   if(elemnt.value=="5")
   {
    elemnt.value="0"; 
   }
   data_string=':0:0000:2\n';
   xhttp.open("GET","data_string?data_string="+data_string,true);
   xhttp.send();
   data_string="";
}

function zero_all()
{
  
   var xhttp = new XMLHttpRequest();
   data_string=':0:0000:3\n';
   document.getElementById("klapan_4").style.backgroundColor = "red";
   document.getElementById("klapan_5").style.backgroundColor = "red";
   document.getElementById("klapan_6").style.backgroundColor = "red";
   document.getElementById("klapan_9").style.backgroundColor = "red";
   document.getElementById("klapan_10").style.backgroundColor = "red";
   document.getElementById("klapan_11").style.backgroundColor = "red";
   document.getElementById("klapan_13").style.backgroundColor = "red";
   xhttp.open("GET","data_string?data_string="+data_string,true);
   xhttp.send();
   alert("ALL TURNED OFF!");
   data_string="";
}
</script>
<script>
function sendData(led) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("Sensor1").innerHTML =
      this.responseText;
    }
  };
  xhttp.open("GET", "setLED?Sensor1="+led, true);
  xhttp.send();
}
 
setInterval(function() {
  // Call a function repetatively with 2 Second interval
  getData1();
  if(document.getElementById("Data").innerHTML=="no connection")
  {
    
    if(s_b.disabled)
      {
    area.disabled=false;
    s_b.disabled=false;
    gas1.disabled=false;
    gas2.disabled=false;
    gas3.disabled=false;
    area.style.backgroundColor='white';
    gas2.style.backgroundColor='#e3e3e3';
    gas1.style.backgroundColor='#e3e3e3';
    gas3.style.backgroundColor='#e3e3e3';
    s_b.style.backgroundColor='#e3e3e3';
    click_ok=false;
       }  
    
    }
  
}, 1000); //2000mSeconds update rate
 
function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("Adres").innerHTML =
      this.responseText;
    }
  };
  xhttp.open("GET", "takeAdress", true);
  xhttp.send();
}

function getData1() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("Data").innerHTML =
      this.responseText;
      
   
    }
  };
  xhttp.open("GET", "takeData", true);
  xhttp.send();
}
</script>
</center>
</body>
</html>
)=====";

String webPageStart="";
ESP8266WebServer webserver(80);
WiFiServer server(8080);
WiFiClient serverClients[MAX_SRV_CLIENTS];
String Adres[MAX_SRV_CLIENTS]="";
int Time_now, Time_last;
void data_adres()
{
  String adres=message;
  webserver.send(200,"text/plane",adres); 
}

void data()
{
messaging();
 webserver.send(200,"text/plane",message); 
 
}

void print_to_stm()
{
  String a= webserver.arg("data_string");
  //uint8_t n = a.length();
  //Serial.print(a);
  String value_to_conv;
  int pressure_to_go=3;
  uint8_t i;
  if(a[0]==':')
  {
    //Serial.print("Hello!");
  
    for(i=3;i<20;i++)
    {
      if(a[i]==':')
      {
        //Serial.print("here");
        break;
        }
      value_to_conv+=a[i];
         
     }
    
    pressure_to_go=value_to_conv.toInt();
    Serial.write('W');
    Serial.write(a[0]);
    Serial.write(a[1]-0x30);
    Serial.write(a[2]);
    Serial.write((pressure_to_go&0xff000000)>>32);
    Serial.write((pressure_to_go&0xff0000)>>16);
    Serial.write((pressure_to_go&0xff00)>>8);
    Serial.write(pressure_to_go&0xff);
    Serial.write(a[i]);
    Serial.write(a[i+1]-0x30);
    Serial.write('\n');
  }
  
  else
  {
    Serial.write('W');
    Serial.write(a[0]-0x30);
    Serial.write(a[1]-0x30);
    Serial.write(a[2]);
    Serial.write(1);
    Serial.write(1);
    Serial.write(1);
    Serial.write(1);
    Serial.write(1);
    Serial.write(1);
    Serial.write('\n'); 
  }
  
  /*for(uint8_t i=0; i<n;i++)
  {
    if((a[i]!=':')&&(a[i]!=';'))
    {
    a[i]=a[i]-0x30;  
    }
    Serial.write(a[i]);
  }*/
 // Serial.println(a);
 // Serial.print(n);
 webserver.send(200, "text/html", "OK");
}

void handle_setup()
{
   webserver.on("/", [](){
   webserver.send(200, "text/html", webPage);
  });
  webserver.on("/data_string", print_to_stm);
 webserver.on("/takeData", data);
   
  
  webserver.begin();
  }

IPAddress staticIP(192, 168, 1, 112); //ESP static ip
IPAddress gateway(192, 168, 1, 1);   //IP Address of your WiFi Router (Gateway)
IPAddress subnet(255, 255, 255, 0);  //Subnet mask
IPAddress dns(8, 8, 8, 8);  //DNS

void setup() {
Serial.begin(115200);
//Serial.println("Start_server");
WiFi.begin(ssid, password);


while (WiFi.status() != WL_CONNECTED) {
    delay(500);
   Serial.print(".");
  }
//Serial.println("");

//Serial.println("");
//Serial.print("Connected to ");  //  "Подключились к "
///Serial.println(ssid);
Serial.print("IP address: ");  //  "IP-адрес: "
Serial.println(WiFi.localIP());
 
/*if (mdns.begin("esp8266", WiFi.localIP())) {
   // Serial.println("MDNS responder started");
               //  "Запущен MDNSresponder"
  }*/
//server.begin();

handle_setup();
Serial.println("HTTP server started");

//webPage = "<h1>Enrice Web Server</h1><table border = 2 ><th>Name_of_device</th><th>first sensor</th><th>Second sensor</th>";
//webPageStart = webPage;
}

void messaging()
{
  Time_now = millis();
  
if(Serial.available())
  {
     if(Serial.read()=='$')
      {  
        for(uint8_t i=0; i<255; i++) message[i]=0;
      for(uint8_t i=0; ;i++)
      {

          message[i]=Serial.read();
         if(message[i]=='*')
          {
            message[i]=0;
           break;
           }
         
          if (!Serial.available())
          {
            break;
          }
      
      }
     }     
 }
}  
     
           
    
    




void loop() {

 /*uint8_t i;
  if (server.hasClient())
  {
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (!serverClients[i] || !serverClients[i].connected())
      {
        if(serverClients[i]) serverClients[i].stop();
        serverClients[i] = server.available();
        continue;
      }
    }
    //no free spot
    WiFiClient serverClient = server.available();
    serverClient.stop();
  }
  for(i = 0; i < MAX_SRV_CLIENTS; i++)
  {
   
    if (serverClients[i] && serverClients[i].connected())
    {
      if(serverClients[i].available())
      {
        char c;
        String str="";
        while(serverClients[i].available())
        {
          c = serverClients[i].read();
          str+=c;
          if(c==',')
          {      
          //Serial.println(str);
       
          } 
          
          
          if(c==';')
          {
            Adres[i]=str;
            Serial.println(Adres[i]);
            str="";
           // Serial.println(str);
          }
       
      } 
    }

  }
/*if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String str="";
    if(client.connected())
    {
      if(client.available())
      {
        char c = client.read();
        str+=c;
        if(c==';')
        {
          Serial.println(str);
          str="";
          }         
        }
      }
 }
  */
//}
  
 
// messaging();
    webserver.handleClient();
    


}
