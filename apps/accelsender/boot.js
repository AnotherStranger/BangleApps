(() => {
  /**
   * KalmanFilter
   * @class
   * @author Wouter Bulten
   * @see {@link http://github.com/wouterbulten/kalmanjs}
   * @version Version: 1.0.0-beta
   * @copyright Copyright 2015-2018 Wouter Bulten
   * @license MIT License
   * @preserve
   */
  class KalmanFilter {
    /**
     * Create 1-dimensional kalman filter
     * @param  {Number} options.R Process noise
     * @param  {Number} options.Q Measurement noise
     * @param  {Number} options.A State vector
     * @param  {Number} options.B Control vector
     * @param  {Number} options.C Measurement vector
     * @return {KalmanFilter}
     */
    constructor({ R = 1, Q = 1, A = 1, B = 0, C = 1 } = {}) {
      this.R = R; // noise power desirable
      this.Q = Q; // noise power estimated

      this.A = A;
      this.C = C;
      this.B = B;
      this.cov = NaN;
      this.x = NaN; // estimated signal without noise
    }

    /**
     * Filter a new value
     * @param  {Number} z Measurement
     * @param  {Number} u Control
     * @return {Number}
     */
    filter(z, u = 0) {
      if (isNaN(this.x)) {
        this.x = (1 / this.C) * z;
        this.cov = (1 / this.C) * this.Q * (1 / this.C);
      } else {
        // Compute prediction
        const predX = this.predict(u);
        const predCov = this.uncertainty();

        // Kalman gain
        const K = predCov * this.C * (1 / (this.C * predCov * this.C + this.Q));

        // Correction
        this.x = predX + K * (z - this.C * predX);
        this.cov = predCov - K * this.C * predCov;
      }

      return this.x;
    }

    /**
     * Predict next value
     * @param  {Number} [u] Control
     * @return {Number}
     */
    predict(u = 0) {
      return this.A * this.x + this.B * u;
    }

    /**
     * Return uncertainty of filter
     * @return {Number}
     */
    uncertainty() {
      return this.A * this.cov * this.A + this.R;
    }

    /**
     * Return the last filtered measurement
     * @return {Number}
     */
    lastMeasurement() {
      return this.x;
    }

    /**
     * Set measurement noise Q
     * @param {Number} noise
     */
    setMeasurementNoise(noise) {
      this.Q = noise;
    }

    /**
     * Set the process noise R
     * @param {Number} noise
     */
    setProcessNoise(noise) {
      this.R = noise;
    }
  }

  const filterX = new KalmanFilter();
  const filterY = new KalmanFilter();
  const filterZ = new KalmanFilter();

  /**
   * Sends a message to the gadgetbridge via Bluetooth.
   * @param {Object} message - The message to be sent.
   */
  function gbSend(message) {
    try {
      Bluetooth.println("");
      Bluetooth.println(JSON.stringify(message));
    } catch (error) {
      console.error("Failed to send message via Bluetooth:", error);
    }
  }

  /**
   * The maximum acceleration data.
   * @type {Object}
   */
  var max_acceleration = {
    x: 0,
    y: 0,
    z: 0,
    mag: 0,
  };

  /**
   * Flag indicating whether acceleration data has been received.
   * @type {boolean}
   */
  var hasData = false;

  function filter(accel) {
    var filtered = {
      x: 0,
      y: 0,
      z: 0,
      mag: 0,
    };
    filtered.x = filterX.filter(accel.x);
    filtered.y = filterY.filter(accel.y);
    filtered.z = filterZ.filter(accel.z);
    filtered.mag = Math.sqrt(
      filtered.x * filtered.x +
        filtered.y * filtered.y +
        filtered.z * filtered.z,
    );

    return filtered;
  }

  /**
   * Updates the maximum acceleration if the current acceleration is greater.
   * @param {Object} accel - The current acceleration object with x, y, z, and mag properties.
   */
  function updateAcceleration(accel) {
    hasData = true;

    const new_accel = filter(accel);
    var current_max_raw = new_accel.mag;
    var max_raw = max_acceleration.mag;
    if (current_max_raw > max_raw) {
      max_acceleration = new_accel;
    }
  }

  /**
   * Updates the acceleration data and sends it to gadgetbridge.
   * Resets the maximum acceleration.
   * Note: If your interval setting is too short, the last value gets sent again.
   */
  function sendAccelerationData() {
    var accel = hasData ? max_acceleration : Bangle.getAccel();

    var update_data = {
      t: "accel",
      accel: accel,
    };
    gbSend(update_data);

    max_acceleration = {
      x: 0,
      y: 0,
      z: 0,
      mag: 0,
    };
    hasData = false;
  }

  /**
   * Configuration object.
   * @type {Object}
   */
  var config = require("Storage").readJSON("accelsender.json") || {};

  if (config.enabled) {
    // Gadgetbridge needs to enable and disable tracking by writing {enabled: true} to "accelsender.json" and reloading
    setInterval(sendAccelerationData, config.interval);
    Bangle.on("accel", updateAcceleration); // Log all acceleration events
  }
})();
