/*  We chose to perform Analog-To-Digital (ADC) conversions by directly manipulating the ATmega328P's SFRs
 *  rather than relying on Arduino's analogRead() function, both as a learning exercise and to poll the sensor as 
 *  quickly as possible.
 *  
 *  Relevant registers:
 *  - PRR: Power Reduction Register
 *     - [3] PRADC: controls whether ADC is enabled(0) or disabled(1)
 *  - ADCSRA: ADC Control and Status Register A
 *     - [2:0] ADPS2:0: sets the prescaler value for the ADC clock
 *     - [6] ADSC: triggers a conversion when set, clears when conversion is complete
 *     - [7] ADEN: controls whether ADC is disabled(0) or enabled(1)
 *  - ADMUX: ADC Multiplexer Selection Register
 *     - [3:0] MUX3:0: controls which ADC pin will be read for conversion
 *     - [5] ADLAR: controls whether values in the ADC Data Register are right(0) or left(1) adjusted
 *     - [7:6] REFS1:0: controls the analog reference voltage
 *  - DIDR0: Digital Input Disable Register 0
 *     - [5:0] ADCnD: controls whether digital input buffer on ADC pin n is enabled(0) or disabled(1)
 *  - ADCW: ADC Data Register
 *     - [7:0] ADCL: the low byte of the result of the latest conversion
 *     - [15:8] ADCH: the high byte of the result of the latest conversion
*/

bool sensor_smoothing = false;
const uint8_t reads_per_smooth = 5;
uint16_t adc_reads_buffer[reads_per_smooth];
uint8_t curr_num_reads = 0;
uint16_t smooth_avg_buffer;
uint8_t low_read_index;
uint8_t high_read_index;
uint16_t diff_from_avg;

void configure_sensor() {
  memset(adc_reads_buffer, 0, sizeof(adc_reads_buffer));
  curr_num_reads = 0;
  
  // ensure ADC isn't already disabled externally to conserve power
  PRR &= ~(1 << PRADC);
  // disable ADC
  ADCSRA &= ~(1 << ADEN);
  // configure ADC to read from ADC0 input channel (A0)
  ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (1 << MUX0));
  // configure analog reference voltage as AVcc w/ ext. capacitor at AREF pin (provided by Arduino)
  ADMUX &= ~(1 << REFS1);
  ADMUX |= (1 << REFS0);
  // configure conversion results to be stored right-adjusted in ADCW
  ADMUX &= ~(1 << ADLAR);
  // set ADC clock prescaler value to 128 (default)
  ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  // disable digital input buffers for all ADC channels to conserve power
  DIDR0 |= ~0;
  // disable ADC interrupts
  ADCSRA &= ~(1 << ADIE);
  // re-enable ADC
  ADCSRA |= (1 << ADEN);
  // perform throwaway conversion (the first conversion after re-enabling ADC takes longer than usual)
  ADCSRA |= (1 << ADSC);
  // wait for conversion to complete
  while ((ADCSRA & (1 << ADSC)) >> ADSC);
}

void enable_sensor_smoothing() {
  sensor_smoothing = true;
}

void disable_sensor_smoothing() {
 sensor_smoothing = false;
}

uint16_t read_sensor() {
  
  if (sensor_smoothing) {
    
    curr_num_reads = 0;
    smooth_avg_buffer = 0;
    diff_from_avg = 0;
    
    while (curr_num_reads < reads_per_smooth) {
      trigger_adc();
      adc_reads_buffer[curr_num_reads] = ADCW;
      curr_num_reads++;
    }

    // for every sensor reading in the buffer...
    for (int r=0; r < reads_per_smooth; r++) {
      smooth_avg_buffer = 0;
      low_read_index = 0;
      high_read_index = 0;

      // average every value besides the current one, and also record the lowest and highest values
      for (int i=0; i < reads_per_smooth; i++) {
        if (i == r) {continue;}
        if (adc_reads_buffer[i] < adc_reads_buffer[low_read_index]) {low_read_index = i;}
        if (adc_reads_buffer[i] > adc_reads_buffer[high_read_index]) {high_read_index = i;}
        smooth_avg_buffer += adc_reads_buffer[i];
      }
      smooth_avg_buffer /= reads_per_smooth - 1;

      // compute the difference between the current value and the average of all the other values
      if (adc_reads_buffer[r] > smooth_avg_buffer) {
        diff_from_avg = adc_reads_buffer[r] - smooth_avg_buffer;
      } else {
        diff_from_avg = smooth_avg_buffer - adc_reads_buffer[r];
      }

      // if that difference is significantly greater than the span of the other values, assume the current value is an outlier and eliminate it
      if (diff_from_avg > (adc_reads_buffer[high_read_index] - adc_reads_buffer[low_read_index]) * 3) {
        adc_reads_buffer[r] = smooth_avg_buffer;
      }
    }

    // return the average reading after discarding the outliers
    smooth_avg_buffer = 0;
    for (int r=0; r < reads_per_smooth; r++) {
      smooth_avg_buffer += adc_reads_buffer[r];
    }
    smooth_avg_buffer /= reads_per_smooth;
    return adc_reads_buffer[0];
    
  } else {

    // take only one sensor reading and return the value
    trigger_adc();
    return ADCW;
  }
}

void trigger_adc() {
  // start an ADC conversion
  ADCSRA |= (1 << ADSC);
  // wait for the conversion to complete
  while ((ADCSRA & (1 << ADSC)) >> ADSC);
}
