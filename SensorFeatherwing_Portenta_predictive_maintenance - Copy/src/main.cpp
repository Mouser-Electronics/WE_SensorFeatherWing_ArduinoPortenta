

#include <Fan_PM_inferencing.h>

#include "sensorBoard.h"

// USB-Serial debug Interface
TypeSerial *SerialDebug;
// WE 3-axis acceleration sensor object
ITDS *sensorITDS;

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static uint32_t run_inference_every_ms = 200;
static rtos::Thread inference_thread(osPriorityLow);
static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
static float inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];



#ifndef LED_BUILTIN
  #define LED_BUILTIN PC13
#endif

/* Forward declaration */
void run_inference_background();

void setup() {

  delay(5000);
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // Using the USB serial port for debug messages
  SerialDebug = SSerial_create(&Serial);
  SSerial_begin(SerialDebug, 115200);

  SSerial_printf(SerialDebug, "Test");

  sensorITDS = ITDSCreate(SerialDebug);
  if (!ITDS_simpleInit(sensorITDS)) {
    SSerial_printf(SerialDebug, "ITDS init failed \r\n");
  }

  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
    ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
    return;
  }

  inference_thread.start(mbed::callback(&run_inference_background));


}

/**void ei_printf(const char *format, ...) {
  static char print_buf[1024] = { 0 };

  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);

  if (r > 0) {
    SSerial_write(SerialDebug, print_buf);
  }
}*/


void run_inference_background()
{
   SSerial_printf(SerialDebug, "Test2");
  // wait until we have a full buffer
  delay((EI_CLASSIFIER_INTERVAL_MS * EI_CLASSIFIER_RAW_SAMPLE_COUNT) + 100);

  // This is a structure that smoothens the output result
  // With the default settings 70% of readings should be the same before classifying.
  ei_classifier_smooth_t smooth;
  ei_classifier_smooth_init(&smooth, 10 /* no. of readings */, 7 /* min. readings the same */, 0.8 /* min. confidence */, 0.3 /* max anomaly */);

  while (1) {
    // copy the buffer
    memcpy(inference_buffer, buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE * sizeof(float));

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(inference_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
      ei_printf("Failed to create signal from buffer (%d)\n", err);
      return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
      ei_printf("ERR: Failed to run classifier (%d)\n", err);
      return;
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
              result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": ");

    // ei_classifier_smooth_update yields the predicted label
    const char *prediction = ei_classifier_smooth_update(&smooth, &result);
    ei_printf("%s ", prediction);
    // print the cumulative results
    ei_printf(" [ ");
    for (size_t ix = 0; ix < smooth.count_size; ix++) {
      ei_printf("%u", smooth.count[ix]);
      if (ix != smooth.count_size + 1) {
        ei_printf(", ");
      }
      else {
        ei_printf(" ");
      }
    }
    ei_printf("]\n");

    //delay(run_inference_every_ms);
  }

  ei_classifier_smooth_free(&smooth);
}


void loop(){
  SSerial_printf(SerialDebug, "Test3");

  while(1){
    // Determine the next tick (and then sleep later)
    uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

    // roll the buffer -3 points so we can overwrite the last one
    numpy::roll(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, -3);



    if (ITDS_readSensorData(sensorITDS)) {
      
      buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3] = (sensorITDS->data[itdsXAcceleration])*9.80665f;
      buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2] = (sensorITDS->data[itdsYAcceleration])*9.80665f;
      buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1] = (sensorITDS->data[itdsZAcceleration])*9.80665f;


    }
    
    // and wait for next tick
    uint64_t time_to_wait = next_tick - micros();
    delay((int)floor((float)time_to_wait / 1000.0f));
    delayMicroseconds(time_to_wait % 1000);
  }
  
}
