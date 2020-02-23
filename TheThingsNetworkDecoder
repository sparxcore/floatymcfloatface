 function Bytes2Float32(bytes) {
    var sign = (bytes & 0x80000000) ? -1 : 1;
    var exponent = ((bytes >> 23) & 0xFF) - 127;
    var significand = (bytes & ~(-1 << 23));
    if (exponent == 128)
        return sign * ((significand) ? Number.NaN : Number.POSITIVE_INFINITY);
    if (exponent == -127) {
        if (significand === 0) return sign * 0.0;
        exponent = -126;
        significand /= (1 << 22);
    } else significand = (significand | (1 << 23)) / (1 << 23);
    return sign * significand * Math.pow(2, exponent);
  }
  function Decoder(bytes, port) {
  var lat = bytes[3] << 24 | bytes[2] << 16 | bytes[1] << 8 | bytes[0];
  var lon = bytes[7] << 24 | bytes[6] << 16 | bytes[5] << 8 | bytes[4];
  var acc = bytes[8];
  var alt = bytes[9];
  var solarV = bytes[11] << 8 | bytes[10];
  var battV = bytes[13] << 8 | bytes[12];
  var airT = bytes[15]<<24>>16 | bytes[14];
  //var airT = bytes[15] << 8 | bytes[14];
  //var waterT = bytes[17] << 8 | bytes[16];
  var waterT = bytes[17]<<24>>16 | bytes[16];
  //var intT = bytes[19] << 8 | bytes[18];
  var intT = bytes[19]<<24>>16 | bytes[18];
  var intH =  bytes[20];
  
  
  return{
    latitude:  Bytes2Float32(lat),
    longitude: Bytes2Float32(lon),
    hdop: (acc/100),
    altitude: alt,
    solarV: (solarV/100),
    batteryV: (battV/100),
    airT: (airT/100),
    waterT: (waterT/100),
    intT: (intT/100),
    intH: intH
  };
  }
