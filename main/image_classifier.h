#ifndef IMAGE_CLASSIFIER_H
#define IMAGE_CLASSIFIER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Teachable Machine Modell-Parameter (Standard)
#define TM_INPUT_WIDTH    224
#define TM_INPUT_HEIGHT   224
#define TM_INPUT_CHANNELS 3      // RGB

// Maximale Anzahl Klassen (anpassen an Ihr Modell!)
#define TM_MAX_CLASSES    5

// Ergebnis einer Klassifikation
typedef struct {
    int class_index;             // Index der erkannten Klasse (0, 1, 2, ...)
    float confidence;            // Konfidenz (0.0 - 1.0)
    const char* class_name;      // Name der Klasse (falls definiert)
} classification_result_t;

// Initialisiert das TFLite Modell
// Rückgabe: true bei Erfolg
bool classifier_init(void);

// Führt Klassifikation auf einem RGB888 Bild durch
// input_rgb888: Zeiger auf RGB888 Bilddaten (TM_INPUT_WIDTH * TM_INPUT_HEIGHT * 3 Bytes)
// result: Zeiger auf Ergebnis-Struktur
// Rückgabe: true bei Erfolg
bool classifier_run(const uint8_t* input_rgb888, classification_result_t* result);

// Gibt alle Klassifikations-Ergebnisse zurück (für Top-N Anzeige)
// results: Array von Ergebnissen
// max_results: Maximale Anzahl Ergebnisse
// Rückgabe: Anzahl der tatsächlichen Ergebnisse
int classifier_run_all(const uint8_t* input_rgb888, classification_result_t* results, int max_results);

// Ressourcen freigeben
void classifier_deinit(void);

#ifdef __cplusplus
}
#endif

#endif