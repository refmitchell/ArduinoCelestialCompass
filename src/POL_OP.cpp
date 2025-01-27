/**
   \file POL_OP.cpp
   \brief Implementation of the POL_OP wrapper class for reading the polarisation 
          opponent PCBs.

   The polarisation opponent PCBs contain two analogue-to-digital converters
   which can be read directly, however, this class was constructed to abstract
   this process to a 'polarisation opponent unit' or POL_OP.

   See POL_OP class documentation.

   \note The ADC we use is the ADS112C04 by Texas Instruments. The
   library for the ADC was adapted from a previous Arduino library for
   the ADS112C04 (112 vs. 122). As a result, the device is erroneously
   called the ADS112C04 throughout the code. I was not able to fix
   this without introducing major changes (which invariably introduce
   major bugs); I was worried about breaking something which I then
   wouldn't have time to test. Please be aware of this difference
   if you plan to modify any of the I2C libraries for this device!

   \author Robert Mitchell
*/

#include <POL_OP.h>


POL_OP::POL_OP(){
  // Default constructor, if used then 'initialise()' must be called!
}

/**
   Constructor. Initialises both ADCs on the board.

   \param wire_port The TWI used by the POL_OP
   \param A2D1_addr The I2C address of the first ADC.
   \param A2D1_addr The I2C address of the second ADC.
   \param debug Debug flag, passed to the ADC library.
   \param channel The I2C multiplexer which this unit is connected to.
*/
POL_OP::POL_OP(TwoWire &wire_port,
               uint8_t A2D1_addr,
               uint8_t A2D2_addr,
               bool debug,
               uint8_t channel, 
               Stream &outstream){
  uint8_t A2D_addrs[2] = {A2D1_addr, A2D2_addr};
  output_stream = &outstream;

  for (int i = 0; i < 2; i++){
    // Debugging output disabled by default
    if (debug)
      A2Ds[i].enableDebugging();

    // Initialise the A2Ds
    if(!(A2Ds[i].begin(A2D_addrs[i], wire_port))){
      output_stream->print("A2D ");
      output_stream->print(i+1);
      output_stream->println(" initialisation error. Exiting.");
      exit(1);
    }

    uint8_t rate = 1; //Default 0; 3 bit code is used to select the datarate, see datasheet
    if(!(A2Ds[i].configureADCmode(ADS112C04_POL_OP_MODE, rate))){
      output_stream->print("A2D ");
      output_stream->print(i+1);
      output_stream->println(" mode configuration error. Exiting.");
      exit(1);
    }

    // Start the ADC. Needed for continuous mode, redundant for single-shot
    A2Ds[i].start();
    A2Ds[i].setInputMultiplexer(ADS112C04_MUX_AIN1_AIN0); //Initialise mux channel
  }

  output_stream->println("POL_OP: Init success");

  // Set mux channel
  this->channel = channel;
}

/**
 * Initialisation, required when using default constructor (e.g. if you want
 * to build an array of POL_OP objects)
 */

void POL_OP::initialise(TwoWire &wire_port,
               uint8_t A2D1_addr,
               uint8_t A2D2_addr,
               bool debug,
               uint8_t channel, 
               Stream &outstream){
  uint8_t A2D_addrs[2] = {A2D1_addr, A2D2_addr};
  output_stream = &outstream;

  for (int i = 0; i < 2; i++){
    // Debugging output disabled by default
    if (debug)
      A2Ds[i].enableDebugging();

    // Initialise the A2Ds
    if(!(A2Ds[i].begin(A2D_addrs[i], wire_port))){
      output_stream->print("A2D ");
      output_stream->print(i+1);
      output_stream->println(" initialisation error. Exiting.");
      exit(1);
    }

    uint8_t rate = 1; //Default 0; 3 bit code is used to select the datarate, see datasheet
    if(!(A2Ds[i].configureADCmode(ADS112C04_POL_OP_MODE, rate))){
      output_stream->print("A2D ");
      output_stream->print(i+1);
      output_stream->println(" mode configuration error. Exiting.");
      exit(1);
    }

    // Start the ADC. Needed for continuous mode, redundant for single-shot
    A2Ds[i].start();
    A2Ds[i].setInputMultiplexer(ADS112C04_MUX_AIN1_AIN0); //Initialise mux channel
  }

  output_stream->println("POL_OP: Init success");

  // Set mux channel
  this->channel = channel;
}

/**
   Read raw data from a single ADC channel.

   \note In addition to the Adafruit I2C mux, each ADC also has its own
   multiplexer, on which two channels are in use (one for each
   photodiode). This function assumes the caller has already
   configured the ADC to read from the desired channel.

   \warning This function was used to read from the polarisation opponent
   units during early prototyping and its current working status is unknown.

   \warning The ADC readout uses a 16-bit encoding which requires sign
   extension which is not provided by this function!

   \param A2D The ADC object we want to read from.
   \param delay A time delay (in ms) used to ensure data integrity.
   \return The raw 16 bit ADC data.
*/
uint32_t POL_OP::read_raw_data(SFE_ADS112C04 A2D, uint8_t t_delay){
  delay(t_delay);
  A2D.start(); // Start conversion

  // Record the start time so we can timeout
  unsigned long start_time = millis();
  bool drdy = false;

  while ((drdy == false) &&
         (millis() < (start_time + ADS112C04_CONVERSION_TIMEOUT))
         ){

    delay(5);
    drdy = A2D.checkDataReady();
  }

  if (drdy == false) {
    output_stream->println("DRDY timed out.");
    return 0; // Error codes?
  }

  // Read raw data
  int32_t raw_ADC_data = A2D.readADC();
  return raw_ADC_data;
}

/**
   Read from the ADC if it is configured to continuous mode.

   \note You must make sure the ADC is in continuous mode before
   assuming this code will work. If initialised in POL_OP_MODE, this
   will be the case. Similarly, you must make sure the ADC channel is
   set prior to attempting a read to ensure sensible data.

   \param A2D The ADC object from which we wish to read.
   \param delay A time delay (in ms) to ensure data integrity.
   \return The raw ADC reading.
*/
uint32_t POL_OP::read_continuous(SFE_ADS112C04 A2D, uint8_t t_delay){
  // The data counter is not in use. This function assumes that the
  // reading will be 'new'.
  delay(t_delay);
  int32_t raw_ADC_data = A2D.readADC();
  return raw_ADC_data;
}

/**
   This function performs one complete read cycle of a specific ADC
   on the PCB.

   \param idx The integer index of the ADC to be read (0 or 1).
   \param readings A pointer to an array large enough to store the output
                   (of size > 2). The readings will be placed in this array.
   \param delay A time delay (in ms) used to ensure data integrity.

   \return True on success, false otherwise.

   \warning The current working status of this function is unknown.
*/
bool POL_OP::read_A2D(uint8_t idx, uint32_t* readings, uint8_t t_delay){
  /*
    Performs one complete read cycle of a specific A2D.
    Results are stored in readings, which should be
    declared by the caller.

    Output array should be large enough for both results.
   */
  if (idx > N_A2Ds - 1) {
    output_stream->println("A2D read err, A2D idx out of range.");
    return false;
  }

  A2Ds[idx].setInputMultiplexer(ADS112C04_MUX_AIN0_AIN1);
  readings[0] = read_continuous(A2Ds[idx], t_delay);
  A2Ds[idx].setInputMultiplexer(ADS112C04_MUX_AIN2_AIN3);
  readings[1] = read_continuous(A2Ds[idx], t_delay);

  return true;
}

/**
   Performs a full (very slow) read from the PCB.

   \note The ADC encodes its output as a 16-bit signed integer which requres
   sign extension to be represented as a standard int. This function performs
   the required sign-extension.

   \warning The working status of this function is unknown. If it does work, it
   is guaranteed to be slow as it does not take advantage of the channel 
   switching delays on the ADC.

   \param readings A pointer to an array (size >= 4) which can hold all of the
                   photodiode readings. The values will be placed in this array.
   \param delay A time delay (in ms) to ensure data integrity.
   \return true on success, false otherwise.
*/
bool POL_OP::read_sensor(int *readings, uint8_t t_delay){
  uint32_t out_1[2];
  uint32_t out_2[2];

  bool r1 = read_A2D(0,out_1,t_delay);
  bool r2 = read_A2D(1,out_2,t_delay);
  if (!(r1 && r2)){
    output_stream->println("Error reading from one or more A2Ds.");
    return false;
  }

  for (int i = 0; i < N_A2Ds; i++){
    // Sign-extended readings
    readings[i] = (((int) out_1[i]) << 16) >> 16;
    readings[i+2] =(((int) out_2[i]) << 16) >> 16;
  }

  // Read successful
  return true;
}


/**
   Perform a full interleaved read from the PCB.

   This PCB read function takes advantage of the delays incurred in
   channel switching on the ADCs by performing one read, changing channel,
   then reading from the other ADC while we wait for the first one to
   finish changing channel.

   \parblock
   \note In the source there is a lot of code dedicated to time tracking.
   This is functionally irrelevant and was only used to characterise the
   sensor read latency. All print-outs should currently be disabled and
   can be re-enabled by uncommenting the print stream below.
   \endparblock

   \parblock
   \note The default delay is 100ms but this can be set as low as 25ms.
         Any delay less than 25ms will start to cause read errors for the
         last polarisation opponent unit in the chain.
   \endparblock

   \param readings Pointer to an array large enough to house all four
                   photodiode readings as integers. The photodiode readings
                   will be stored here.
   \param delay Time delay (in ms) used to ensure data integrity.
   \return true on success, false otherwise
*/
bool POL_OP::read_sensor_interleaved(int *readings, uint8_t t_delay){
  uint32_t out_1[2] = {0,0};
  uint32_t out_2[2] = {0,0};

  // Reads are interleaved to give the multiplexers a chance to change.
  out_1[0] = read_continuous(A2Ds[0], t_delay);
  A2Ds[0].setInputMultiplexer(ADS112C04_MUX_AIN2_AIN3);
  out_2[0] = read_continuous(A2Ds[1], t_delay);

  A2Ds[1].setInputMultiplexer(ADS112C04_MUX_AIN2_AIN3);
  out_1[1] = read_continuous(A2Ds[0], t_delay);
  out_2[1] = read_continuous(A2Ds[1], t_delay);

  // Reset multiplexers to starting position
  A2Ds[0].setInputMultiplexer(ADS112C04_MUX_AIN1_AIN0);
  A2Ds[1].setInputMultiplexer(ADS112C04_MUX_AIN1_AIN0);

  for (int i = 0; i < N_A2Ds; i++){
    // Sign-extended readings
    readings[i] = (((int) out_1[i]) << 16) >> 16;
    readings[i+2] =(((int) out_2[i]) << 16) >> 16;
  }

  return true;
}

bool POL_OP::read_sensor_half(int *readings, uint8_t t_delay=100){
  //uint32_t out_1[2] = {0,0};
  uint32_t out_2[2] = {0,0}; // Think these are the horizontal readings

  // Reads are interleaved to give the multiplexers a chance to change.
  //out_1[0] = read_continuous(A2Ds[0], t_delay);
  //A2Ds[0].setInputMultiplexer(ADS112C04_MUX_AIN2_AIN3);
  out_2[0] = read_continuous(A2Ds[1], t_delay);
  A2Ds[1].setInputMultiplexer(ADS112C04_MUX_AIN2_AIN3);
  //out_1[1] = read_continuous(A2Ds[0], t_delay);
  out_2[1] = read_continuous(A2Ds[1], t_delay);

  // Reset multiplexers to starting position
  //A2Ds[0].setInputMultiplexer(ADS112C04_MUX_AIN1_AIN0);
  A2Ds[1].setInputMultiplexer(ADS112C04_MUX_AIN1_AIN0);

  //Temp
  readings[0] = out_2[0];
  readings[1] = out_2[1];

  // for (int i = 0; i < N_A2Ds; i++){
  //   // Sign-extended readings
  //   readings[i] = (((int) out_1[i]) << 16) >> 16;
  //   readings[i+2] =(((int) out_2[i]) << 16) >> 16;
  // }

  return true;
}

/**
   Get the I2C mux channel information for this polarisation opponent unit.

   \note This is referring to the sensor-level mux (i.e. the Adafruit one),
   \b not the ADC muxes.

   \return The I2C mux channel to which this pol op unit is connected.
*/
uint8_t POL_OP::get_channel(){ return this->channel; }

/**
  Set the PGA gain for both ADCs on the PCB.

  \note The gain must be a valid 3-bit gain setting for the
  ADS112C04. Use relevant definitions in ADS112C04_ADC_PI.hpp
  (ADS112C04_GAIN_X) to ensure valid settings are used. No error
  checking is included!

  \param The desired 3-bit gain setting.

  \return true on success, false on failure
 */
bool POL_OP::set_gain(uint8_t gain){
  bool ret = true;
  for (int i = 0; i < N_A2Ds; i++) {
    // For each A2D get the most recent initParams, update the
    // gain and then reinitialise the chip.
    ADS112C04_initParam* initParams = A2Ds[i].getCurrentInitParams();
    initParams->gainLevel = gain;
    ret = ret && A2Ds[i].reinitialise();
    if (!ret) { 
      output_stream->print("Reinitialisation error with ADC ");
      output_stream->println(i+1);
    }
  }

  if (ret) {
    output_stream->print("POL_OP: Gain set to ");
    output_stream->println((int) gain);
  }
  return ret;
}


/**
   Set data rate using presets defined in ADS112_ADC_PI.hpp;
   definitions included below for convenience:

   \verbatim
   #define ADS112C04_DATA_RATE_20SPS   0x0
   #define ADS112C04_DATA_RATE_45SPS   0x1
   #define ADS112C04_DATA_RATE_90SPS   0x2
   #define ADS112C04_DATA_RATE_175SPS  0x3
   #define ADS112C04_DATA_RATE_330SPS  0x4
   #define ADS112C04_DATA_RATE_600SPS  0x5
   #define ADS112C04_DATA_RATE_1000SPS 0x6
   \endverbatim

   See the datasheet for the \b ADS112C04 for more information.

   \param The desired data rate.
   \return true on success, false on failure.
*/
bool POL_OP::set_data_rate(uint8_t rate){
  bool ret = true;
  for (int i = 0; i < N_A2Ds; i++) {
    // For each A2D get the most recent initParams, update the
    // gain and then reinitialise the chip.
    ADS112C04_initParam* initParams = A2Ds[i].getCurrentInitParams();
    initParams->dataRate = rate;
    ret = ret && A2Ds[i].reinitialise();
    if (!ret) {
      output_stream->print("Reinitialisation error with ADC ");
      output_stream->println(i+1);
    }
  }

  if (ret) {
    output_stream->print("POL_OP: Data rate set to ");
    output_stream->println((int) rate);
  }
  return ret;
}
