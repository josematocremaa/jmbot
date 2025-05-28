#include "mbed.h"
#include <cstdlib>
#include "Servo.h"




Servo hombro(D9);
Servo codo(D5);
Servo muneca(D3);
Servo muneca2(D6);
Servo pinza(D10);

DigitalOut dirPin(D7);
DigitalOut stepPin(D4);

const float resolution = 0.9;
int angle=0;
int turning_time=1000000;

BufferedSerial pc(USBTX, USBRX, 19200);
int ang1=0;
int ang2=0;
int ang3=0;
int ang4=0;
int ang5=0;
int hola=0;


void move(int angle, int turning_time) {
 dirPin=1;
 angle=angle * 3.0526;  //58 /19
 int steps = angle  / resolution;
 
 int ms = (turning_time * resolution) / (2 * angle);

 for (int i = 0; i < steps; i++) {
    stepPin=1;
    wait_us(ms); // Demora de 2 segundos
    stepPin=0;
    wait_us(ms); // Demora de 2 segundos
}
wait_us(500000); // Demora de 2 segundos
 }


struct serial_msg
{
    uint8_t data[26]; // make room for 4*5 bytes  + 2 bytes (frame counter)
    int size;
};

//funciones

void readMessage(const char * buffer, serial_msg & msg)
{
    msg.size = 0;
    char c = '0';
    int index = 0;
    int start = -1;
    int end = -1;
    bool hitOnce = false;
    printf("buffer: %s\n", buffer);
    while (true)
    {
        c = buffer[index++];

        if (c == '<')
        {
            
            start = index;
            continue;
        }
        else if (c == '>' && start != -1)
        {
            end = index;
            break;
        }
        else 
         {
             memcpy(msg.data + msg.size, &c, 1);
             printf("Caracter copiado: %c | Tamano actual: %d\r\n", c, msg.size);
             //printf("msg %s", msg.data);
             msg.size++;
         }
        if (index==26)
        {
            if (hitOnce)
            {
                break;
            }
            else
            {
               index=0;
               hitOnce = true;
            }
        }
    }
    printf("start: %d end: %d\n", start,end);
}


//<-234+678912345678934>


// Create a bufferedSerial object with a 9600 baud rate.


int main(void)
{
    serial_msg msg_in;
    char c;
    char buffer[26]={0};
    int index = 0;
    int size=0;
    char num1_str[5]= {0};
    char num2_str[5]= {0};
    char num3_str[5]= {0};
    char num4_str[5]= {0};
    char num5_str[5]= {0};
    char num6_str[5]= {0};
   // pc.write("Iniciando comunicacion...\r\n", 26);  // Enviar un mensaje al PC
    codo.calibrate(0.0009, 180.0);
    muneca.calibrate(0.0009, 180.0);
    muneca2.calibrate(0.0009, 180.0);
    pinza.calibrate(0.0009, 180.0);
    hombro.calibrate(0.001, 135.0);
    

while (true)
{
    memset(buffer, 0, sizeof(buffer));  // Limpiar buffer antes de leer
    memset(msg_in.data, 0, sizeof(msg_in.data));  // Limpiar mensaje anterior

                
            
                //ThisThread::sleep_for(1s);
            //move(-90, turning_time);  // Regresar
                //ThisThread::sleep_for(1s);
    while ((size = pc.read(buffer, 26)) > 0)
    {   // Si hay datos disponibles
        
        readMessage(buffer, msg_in);
        // pc.write("Buffer recibido: ", 17);
        // pc.write(buffer, size);
        // pc.write("\r\n", 2);
            

             if (msg_in.size>=24) //no cuento <> 
                {  // Fin de mensaje
                       
                    // pc.write("Recibido: ", 10);
                    // pc.write(msg_in.data, msg_in.size);

                    // Extraer los primeros 4 caracteres y convertirlos a un número
                    memcpy(num1_str, msg_in.data, 4);
                    angle = atoi(num1_str);
                    move(angle, turning_time);
                    printf( "Num1: %d, \r\n", angle);

                    // Extraer los siguientes 4 caracteres y convertirlos a un número
                    memcpy(num2_str, msg_in.data + 4, 4);
                    ang1 = atoi(num2_str);
                    hombro.position(ang1);
                    //printf( "Num2: %d, \r\n", ang1);

                   // Extraer los siguientes 4 caracteres y convertirlos a un número
                    memcpy(num3_str, msg_in.data + 8, 4);
                    ang2 = atoi(num3_str);
                    codo.position(ang2);
                   // printf( "Num3: %d, \r\n", ang2);

                   // Extraer los siguientes 4 caracteres y convertirlos a un número
                    memcpy(num4_str, msg_in.data + 12, 4);
                    ang3 = atoi(num4_str);
                    muneca.position(ang3);
                   // printf( "Num4: %d, \r\n", ang3);

                   // Extraer los siguientes 4 caracteres y convertirlos a un número
                    memcpy(num5_str, msg_in.data + 16, 4);
                    ang4 = atoi(num5_str);
                    muneca2.position(ang5);
                    //printf( "Num5: %d, \r\n", ang4);
                    // Extraer los siguientes 4 caracteres y convertirlos a un número
                    memcpy(num6_str, msg_in.data + 20, 4);
                    ang5 = atoi(num6_str);
                    pinza.position(ang5);
                    //printf( "Num5: %d, \r\n", ang5);
                }
         rtos::ThisThread::sleep_for(1s);
    }              
   // <-234+678+123-567+567>
   //<-490-435-436+543-465>
}

}
