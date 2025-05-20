import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import Float64MultiArray

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Configurar el puerto serie
        self.puerto = "/dev/ttyACM0"  # Cambiar a COM9 si usas Windows
        self.baudRate = 9600
        self.ser = serial.Serial(self.puerto, self.baudRate, timeout=1)
        time.sleep(2)  # Esperar que la conexión serial se estabilice

        # Suscriptor de ROS 2 al tópico 'topic' que recibe un array de 5 números
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        """Convierte el array recibido en el formato deseado y lo envía por serial."""
        # Convertir el array en la cadena con el formato <+xxx-xxx+xxx-xxx+xxx>
        mensaje_formateado = self.formatear_mensaje(msg.data)

        # Enviar el mensaje por el puerto serial
        self.ser.write(mensaje_formateado.encode())

        # Mostrar en consola
        self.get_logger().info(f'Recibido: {list(msg.data)}')  # Convertir a lista para imprimir bien
        self.get_logger().info(f'Enviando: {mensaje_formateado}')

    def formatear_mensaje(self, data):
        """Convierte un array de 5 números en la cadena <+xxx-xxx+xxx-xxx+xxx>."""
        cadena = "<"
        for num in data:
            signo = "+" if num >= 0 else "-"  # Determinar el signo
            numero = abs(int(num))  # Convertir a entero positivo sin decimales
            cadena += f"{signo}{numero:03}"  # Asegurar 3 dígitos con ceros a la izquierda
        cadena += ">"
        return cadena

def main(args=None):
    rclpy.init(args=args)
    
    minimal_subscriber = JointController()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
