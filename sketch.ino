#include <Adafruit_GFX.h>
#include <Adafruit_FT6206.h>
#include <Adafruit_ILI9341.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

// Definirea pinilor pentru ecranul TFT si touchscreen
#define PIN_CS 8
#define PIN_RST 9
#define PIN_DC 10
#define PIN_MOSI 11
#define PIN_SCK 12
#define PIN_MISO 13

// Crearea obiectelor pentru ecranul TFT si touchscreen
Adafruit_ILI9341 tft = Adafruit_ILI9341(PIN_CS, PIN_DC, PIN_MOSI, PIN_SCK, PIN_RST, PIN_MISO);
Adafruit_FT6206 ts = Adafruit_FT6206();

// Structura pentru reprezentarea unei perechi de coordonate
struct Pair {
  float x, y;

  // Supradefinirea operatorilor pentru a facilita operatiile cu structura Pair
  const Pair operator+(const Pair &p) {
    return {x + p.x, y + p.y};
  }
  const Pair operator-(const Pair &p) {
    return {x - p.x, y - p.y};
  }
  const Pair operator*(float c) {
    return {c * x, c * y};
  }
  Pair& operator+=(const Pair &p) {
    this->x += p.x;
    this->y += p.y;
    return *this;
  }
  bool operator!=(const Pair &p) {
    return round(x) != round(p.x) || round(y) != round(p.y);
  }

  // Metoda pentru limitarea valorilor intr-un interval dat
  void clamp(float minVal, float maxVal) {
    x = max(min(x, maxVal), minVal);
    y = max(min(y, maxVal), minVal);
  }

  // Metoda pentru calcularea normei (distantei) punctului
  float norm() {
    return sqrt(x * x + y * y);
  }
};

// Definirea unui numar maxim de puncte si declararea variabilelor globale
#define MAX_POINTS 100
int pointCount;
Pair trajectory[MAX_POINTS], position, lastPosition;

// Definirea culorilor pentru desenare
uint16_t colors[] = {ILI9341_RED, ILI9341_GREEN, ILI9341_BLUE, ILI9341_YELLOW, ILI9341_MAGENTA, ILI9341_CYAN};
int colorIndex = 0;

// Crearea semafoarelor pentru sincronizare
SemaphoreHandle_t drawSemaphore, pidStartSemaphore, pidTrackSemaphore, showTrajectorySemaphore;

// Task-ul pentru desenarea traiectoriei
void drawTask(void *pvParameters) {
  while (1) {
    // Asteapta semaforul pentru a incepe desenarea
    xSemaphoreTake(drawSemaphore, portMAX_DELAY);

    // Verifica daca a fost atins ecranul
    if (ts.touched()) {
      pointCount = 0;

      // Deseneaza traiectoria punct cu punct
      while (ts.touched()) {
        TS_Point p = ts.getPoint();
        if (pointCount < MAX_POINTS) {
          trajectory[pointCount].x = p.x;
          trajectory[pointCount].y = p.y;
          tft.fillCircle(p.x, p.y, 2, colors[colorIndex]);
          pointCount++;
        }
        else {
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // Pauza pentru a nu suprasolicita procesorul
      }

      // Schimbarea culorii pentru urmatoarea traiectorie
      colorIndex++;
      if (colorIndex > 5) {
        colorIndex = 0;
      }

      // Activeaza semafoarele pentru a incepe calculul PID si urmarirea traiectoriei
      xSemaphoreGive(pidStartSemaphore);
      xSemaphoreGive(pidTrackSemaphore);
    }
    else {
      xSemaphoreGive(drawSemaphore);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Task-ul pentru controlul PID al miscarii
void pidControlTask(void *pvParameters) {
  float Kp = 0.01, Ki = 0.001, Kd = 0.01;
  Pair error, integralError, derivativeError, lastError, delta;
  while (1) {
    // Asteapta semaforul pentru a incepe calculul PID
    xSemaphoreTake(pidStartSemaphore, portMAX_DELAY);

    position = trajectory[0];
    integralError = {0, 0};
    lastError = {0, 0};

    // Parcurge toate punctele din traiectorie
    for (int i = 0; i < pointCount; i++) {
      do {
        // Asteapta semaforul pentru a continua urmarirea traiectoriei
        xSemaphoreTake(pidTrackSemaphore, portMAX_DELAY);

        // Calculeaza eroarea fata de pozitia curenta
        error = trajectory[i] - position;
        integralError += error;
        integralError.clamp(-10, 10);
        derivativeError = error - lastError;

        // Calculeaza corectia folosind formula PID
        delta = error * Kp + integralError * Ki + derivativeError * Kd;
        delta.clamp(-5, 5);

        lastPosition = position;
        position += delta; // Actualizeaza pozitia curenta
        lastError = error;

        // Daca pozitia curenta nu este aceeasi cu ultima (la nivel de pixel), atunci deseneaza traiectoria
        if (position != lastPosition) {
          xSemaphoreGive(showTrajectorySemaphore);
        }
        else {
          xSemaphoreGive(pidTrackSemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(2));
      } while (error.norm() > 0.5);  // Continua pana cand eroarea devine suficient de mica
    }

    // Elibereaza semaforul pentru a permite desenarea unei alte traiectorii
    xSemaphoreGive(drawSemaphore);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Task-ul pentru afisarea traiectoriei pe ecran
void showTrajectoryTask(void *pvParameters) {
  while (1) {
    // Asteapta semaforul pentru a desena linia
    xSemaphoreTake(showTrajectorySemaphore, portMAX_DELAY);

    // Deseneaza linia intre pozitia anterioara si pozitia curenta
    tft.drawLine(round(lastPosition.x), round(lastPosition.y), round(position.x), round(position.y), colors[colorIndex]);

    // Elibereaza semaforul pentru continuarea urmaririi
    xSemaphoreGive(pidTrackSemaphore);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void setup() {
  Serial.begin(9600);
  tft.begin();  // Initializarea ecranului TFT
  ts.begin();   // Initializarea touchscreen-ului
  tft.setRotation(2);  // Seteaza orientarea ecranului

  // Crearea semafoarelor
  drawSemaphore = xSemaphoreCreateBinary();
  pidStartSemaphore = xSemaphoreCreateBinary();
  pidTrackSemaphore = xSemaphoreCreateBinary();
  showTrajectorySemaphore = xSemaphoreCreateBinary();

  // Crearea task-urilor
  xTaskCreate(drawTask, "Draw", 1000, NULL, 1, NULL);
  xTaskCreate(pidControlTask, "PIDControl", 1000, NULL, 1, NULL);
  xTaskCreate(showTrajectoryTask, "ShowTrajectory", 1000, NULL, 1, NULL);

  // Initializarea semaforului de desenare
  xSemaphoreGive(drawSemaphore);
}

void loop() {}
