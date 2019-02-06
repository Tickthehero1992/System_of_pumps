
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
Serial.write("Hui");
}
int i=0x31;
void loop() {

  // put your main code here, to run repeatedly:
  Serial.write('W');
  Serial.write(':');
    Serial.write('1'-0x30);
    Serial.write(';');
    Serial.write((320002&0xff000000)>>32);
    Serial.write((320002&0xff0000)>>16);
    Serial.write((320002&0xff00)>>8);
    Serial.write(320002&0xff);
    Serial.write(';');
    Serial.write('0'-0x30);
    Serial.write('\n');

    Serial.write('W');
     Serial.write(':');
    Serial.write('0'-0x30);
    Serial.write(';');
    Serial.write((320002&0xff000000)>>32);
    Serial.write((320002&0xff0000)>>16);
    Serial.write((320002&0xff00)>>8);
    Serial.write(320002&0xff);
    Serial.write(';');
    Serial.write('1'-0x30);
    Serial.write('\n');
    i++;
}
