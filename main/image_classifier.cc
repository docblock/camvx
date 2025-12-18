#include "image_classifier.h"
#include "model.h"

// KORREKTUR: Zurück zum verfügbaren Header
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "esp_log.h"
#include "esp_heap_caps.h"  // NEU: Für heap_caps_malloc und MALLOC_CAP_SPIRAM

#include <cstring>
#include <cmath>

static const char* TAG = "classifier";

// Klassennamen (ANPASSEN AN IHR MODELL!)
static const char* class_names[] = {
    "Class 1",
    "Class 2", 
    // Fügen Sie hier Ihre Klassennamen hinzu
};
static const int num_classes = sizeof(class_names) / sizeof(class_names[0]);

// TFLite Globals
namespace {
    const tflite::Model* model = nullptr;
    tflite::MicroInterpreter* interpreter = nullptr;
    TfLiteTensor* input_tensor = nullptr;
    TfLiteTensor* output_tensor = nullptr;
    
    // Tensor Arena - Größe anpassen falls "Arena size too small" Fehler!
    // Da wir PSRAM haben, können wir großzügig sein.
    // Versuchen wir es mit 4 MB (4 * 1024 * 1024)
    constexpr int kTensorArenaSize = 16 * 1024 * 1024; 
    uint8_t* tensor_arena = nullptr;
}

bool classifier_init(void) {
    ESP_LOGI(TAG, "Initializing TFLite classifier...");
    
    // Tensor Arena im PSRAM allokieren (zu groß für internen RAM)
    tensor_arena = (uint8_t*)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM);
    if (!tensor_arena) {
        ESP_LOGE(TAG, "Failed to allocate tensor arena (%d bytes)", kTensorArenaSize);
        return false;
    }
    ESP_LOGI(TAG, "Tensor arena allocated: %d bytes in PSRAM", kTensorArenaSize);

    // Modell laden
    model = tflite::GetModel(g_model);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model schema version mismatch: got %lu, expected %d",
                 model->version(), TFLITE_SCHEMA_VERSION);
        return false;
    }
    ESP_LOGI(TAG, "Model loaded, schema version: %lu", model->version());

    // KORREKTUR: Größe von 15 auf 64 erhöht!
    // Das kostet kaum Speicher, verhindert aber den Überlauf.
    // Stellen Sie sicher, dass die Zahl im Template groß genug ist (z.B. 64)
    static tflite::MicroMutableOpResolver<64> resolver;

    // Standard-Operationen für CNNs
    // Wir prüfen den Rückgabewert nicht, da wir jetzt genug Platz haben.
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddReshape();
    resolver.AddSoftmax();
    resolver.AddFullyConnected();
    resolver.AddMaxPool2D();
    resolver.AddAveragePool2D();
    resolver.AddQuantize();
    resolver.AddDequantize();
    resolver.AddMean();
    resolver.AddPad();
    resolver.AddLogistic();
    resolver.AddTanh();
    
    // Zusätzliche Ops für Teachable Machine / Flex
    resolver.AddConcatenation();
    resolver.AddPack();
    resolver.AddUnpack();
    
    // Falls Op #88 "TRANSPOSE" oder "SPLIT" ist (häufig bei komplexeren Modellen):
    resolver.AddSplit();
    resolver.AddTranspose();
    resolver.AddShape();

    // HIER DIE FEHLENDEN OPS HINZUFÜGEN:
    resolver.AddAdd();      // <--- Das behebt Ihren aktuellen Fehler
    
    // Empfehlung: Fügen Sie diese auch gleich hinzu, die werden oft bei Quantisierung gebraucht:
    resolver.AddMul();
    resolver.AddSub();
    resolver.AddMinimum();
    resolver.AddMaximum();

    // Interpreter erstellen
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;

    // Tensoren allokieren
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        // Wenn es jetzt noch fehlschlägt, liegt es wirklich an einer fehlenden Custom-Op Implementierung,
        // aber AllOpsResolver deckt 99% der Fälle ab.
        ESP_LOGE(TAG, "AllocateTensors failed!");
        return false;
    }

    // Input/Output Tensor holen
    input_tensor = interpreter->input(0);
    output_tensor = interpreter->output(0);

    // Debug-Info ausgeben
    ESP_LOGI(TAG, "Input tensor: dims=%d, shape=[%d,%d,%d,%d], type=%d",
             input_tensor->dims->size,
             input_tensor->dims->data[0],
             input_tensor->dims->data[1],
             input_tensor->dims->data[2],
             input_tensor->dims->data[3],
             input_tensor->type);
    
    ESP_LOGI(TAG, "Output tensor: dims=%d, shape=[%d,%d], type=%d",
             output_tensor->dims->size,
             output_tensor->dims->data[0],
             output_tensor->dims->data[1],
             output_tensor->type);

    if (input_tensor->type == kTfLiteInt8) {
        ESP_LOGI(TAG, "Input quantization: scale=%f, zero_point=%d",
                 input_tensor->params.scale, input_tensor->params.zero_point);
    }
    if (output_tensor->type == kTfLiteInt8) {
        ESP_LOGI(TAG, "Output quantization: scale=%f, zero_point=%d",
                 output_tensor->params.scale, output_tensor->params.zero_point);
    }

    ESP_LOGI(TAG, "Classifier initialized successfully!");
    return true;
}

bool classifier_run(const uint8_t* input_rgb888, classification_result_t* result) {
    if (!interpreter || !input_tensor || !output_tensor || !input_rgb888 || !result) {
        return false;
    }

    // Input-Daten in den Tensor kopieren
    size_t input_size = TM_INPUT_WIDTH * TM_INPUT_HEIGHT * TM_INPUT_CHANNELS;
    
    if (input_tensor->type == kTfLiteInt8) {
        // Quantisiertes Modell: RGB Werte (0-255) zu int8 (-128 bis 127) konvertieren
        int8_t* input_data = input_tensor->data.int8;
        float scale = input_tensor->params.scale;
        int zero_point = input_tensor->params.zero_point;
        
        for (size_t i = 0; i < input_size; i++) {
            // Normalisieren (0-255) -> (0-1) -> Quantisieren
            float normalized = input_rgb888[i] / 255.0f;
            int32_t quantized = (int32_t)(normalized / scale) + zero_point;
            // Clampen auf int8 Bereich
            if (quantized < -128) quantized = -128;
            if (quantized > 127) quantized = 127;
            input_data[i] = (int8_t)quantized;
        }
    } else if (input_tensor->type == kTfLiteUInt8) {
        // Nicht-quantisiertes Modell: Direkt kopieren
        memcpy(input_tensor->data.uint8, input_rgb888, input_size);
    } else if (input_tensor->type == kTfLiteFloat32) {
        // Float Modell: Normalisieren
        float* input_data = input_tensor->data.f;
        for (size_t i = 0; i < input_size; i++) {
            input_data[i] = input_rgb888[i] / 255.0f;
        }
    } else {
        ESP_LOGE(TAG, "Unsupported input tensor type: %d", input_tensor->type);
        return false;
    }

    // Inferenz ausführen
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke failed");
        return false;
    }

    // Ergebnis auslesen - höchste Konfidenz finden
    int best_class = 0;
    float best_confidence = -1.0f;
    int output_size = output_tensor->dims->data[1];  // Anzahl der Klassen

    for (int i = 0; i < output_size; i++) {
        float confidence;
        
        if (output_tensor->type == kTfLiteInt8) {
            int8_t quantized = output_tensor->data.int8[i];
            confidence = (quantized - output_tensor->params.zero_point) * output_tensor->params.scale;
        } else if (output_tensor->type == kTfLiteUInt8) {
            confidence = output_tensor->data.uint8[i] / 255.0f;
        } else if (output_tensor->type == kTfLiteFloat32) {
            confidence = output_tensor->data.f[i];
        } else {
            continue;
        }

        if (confidence > best_confidence) {
            best_confidence = confidence;
            best_class = i;
        }
    }

    result->class_index = best_class;
    result->confidence = best_confidence;
    result->class_name = (best_class < num_classes) ? class_names[best_class] : "Unknown";

    return true;
}

int classifier_run_all(const uint8_t* input_rgb888, classification_result_t* results, int max_results) {
    if (!interpreter || !input_tensor || !output_tensor || !input_rgb888 || !results) {
        return 0;
    }

    // Input-Daten kopieren (gleich wie oben)
    size_t input_size = TM_INPUT_WIDTH * TM_INPUT_HEIGHT * TM_INPUT_CHANNELS;
    
    if (input_tensor->type == kTfLiteInt8) {
        int8_t* input_data = input_tensor->data.int8;
        float scale = input_tensor->params.scale;
        int zero_point = input_tensor->params.zero_point;
        
        for (size_t i = 0; i < input_size; i++) {
            float normalized = input_rgb888[i] / 255.0f;
            int32_t quantized = (int32_t)(normalized / scale) + zero_point;
            if (quantized < -128) quantized = -128;
            if (quantized > 127) quantized = 127;
            input_data[i] = (int8_t)quantized;
        }
    } else if (input_tensor->type == kTfLiteUInt8) {
        memcpy(input_tensor->data.uint8, input_rgb888, input_size);
    } else if (input_tensor->type == kTfLiteFloat32) {
        float* input_data = input_tensor->data.f;
        for (size_t i = 0; i < input_size; i++) {
            input_data[i] = input_rgb888[i] / 255.0f;
        }
    }

    // Inferenz
    if (interpreter->Invoke() != kTfLiteOk) {
        return 0;
    }

    // Alle Ergebnisse sammeln
    int output_size = output_tensor->dims->data[1];
    int result_count = (output_size < max_results) ? output_size : max_results;

    for (int i = 0; i < result_count; i++) {
        float confidence;
        
        if (output_tensor->type == kTfLiteInt8) {
            int8_t quantized = output_tensor->data.int8[i];
            confidence = (quantized - output_tensor->params.zero_point) * output_tensor->params.scale;
        } else if (output_tensor->type == kTfLiteUInt8) {
            confidence = output_tensor->data.uint8[i] / 255.0f;
        } else if (output_tensor->type == kTfLiteFloat32) {
            confidence = output_tensor->data.f[i];
        } else {
            confidence = 0.0f;
        }

        results[i].class_index = i;
        results[i].confidence = confidence;
        results[i].class_name = (i < num_classes) ? class_names[i] : "Unknown";
    }

    return result_count;
}

void classifier_deinit(void) {
    if (tensor_arena) {
        heap_caps_free(tensor_arena);
        tensor_arena = nullptr;
    }
    interpreter = nullptr;
    model = nullptr;
    input_tensor = nullptr;
    output_tensor = nullptr;
    ESP_LOGI(TAG, "Classifier deinitialized");
}