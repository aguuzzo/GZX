# **Robot Seguidor de Líneas**

Este proyecto implementa un robot seguidor de líneas basado en un Arduino Nano, 6 sensores CNY70, motores Pololu y un driver de motores. El robot utiliza un algoritmo de control para seguir de forma eficiente una línea negra sobre un fondo blanco.

---

## **Características**

- **Sensores:** 6 sensores CNY70 para detección precisa de la línea.  
- **Controlador:** Arduino Nano para procesamiento y control.  
- **Motores:** Motores Pololu de corriente continua para el movimiento.  
- **Driver de Motores:** Permite controlar la dirección y velocidad de los motores.  
- **Algoritmo de Control:** Basado en control proporcional (P) o proporcional-derivativo (PD) para seguir la línea.

---

## **Componentes**

- **Microcontrolador:** Arduino Nano (ATmega328P).  
- **Sensores:** 6 sensores ópticos CNY70.  
- **Motores:** Motores Pololu con caja reductora.  
- **Driver de Motores:** L298N o TB6612FNG.  
- **Fuente de Alimentación:** Batería de 7.4V (para los motores) y regulador de 5V para el Arduino y sensores.  
- **Estructura:** Chasis con diseño ligero y estable.  

---

## **Conexiones**

### **Sensores CNY70**  
Conectados a las entradas analógicas del Arduino:  
- **A0, A1, A2, A3, A4, A5.**  

### **Driver de Motores**  
Pines del Arduino conectados al driver:  
- **ENA:** Pin 9 (Control de velocidad motor izquierdo).  
- **IN1, IN2:** Pines 7 y 6 (Control dirección motor izquierdo).  
- **ENB:** Pin 10 (Control de velocidad motor derecho).  
- **IN3, IN4:** Pines 5 y 4 (Control dirección motor derecho).  

---

## **Algoritmo Implementado**

1. Leer los valores de los sensores (valores analógicos).  
2. Convertir los valores a binarios según un umbral.  
3. Calcular la posición de la línea en relación con el centro de los sensores.  
4. Determinar el error y ajustar la velocidad de los motores mediante un controlador proporcional (P).  
5. Enviar comandos al driver para mover los motores.

---

## **Código**

### **Ejemplo: Controlador Proporcional**
```cpp
#include <Arduino.h>

// Pines de sensores
const int sensores[6] = {A0, A1, A2, A3, A4, A5};

// Pines del driver
const int ENA = 9; // Velocidad motor izquierdo
const int IN1 = 7;
const int IN2 = 6;
const int ENB = 10; // Velocidad motor derecho
const int IN3 = 5;
const int IN4 = 4;

int lecturaSensores[6];
int posicion = 0;
int error = 0;
int velocidadBase = 150;
float Kp = 10; // Constante proporcional

void setup() {
  for (int i = 0; i < 6; i++) pinMode(sensores[i], INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // Leer sensores
  posicion = 0;
  for (int i = 0; i < 6; i++) {
    lecturaSensores[i] = analogRead(sensores[i]) > 500 ? 1 : 0; // Umbral 500
    posicion += lecturaSensores[i] * (i - 2); // Peso relativo de cada sensor
  }

  // Calcular error
  error = posicion;

  // Control proporcional
  int ajuste = Kp * error;

  // Calcular velocidades
  int velocidadIzquierda = velocidadBase - ajuste;
  int velocidadDerecha = velocidadBase + ajuste;

  // Controlar motores
  moverMotor(ENA, IN1, IN2, velocidadIzquierda);
  moverMotor(ENB, IN3, IN4, velocidadDerecha);
  delay(10);
}

void moverMotor(int enable, int in1, int in2, int velocidad) {
  if (velocidad > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    velocidad = -velocidad;
  }
  analogWrite(enable, constrain(velocidad, 0, 255));
}
