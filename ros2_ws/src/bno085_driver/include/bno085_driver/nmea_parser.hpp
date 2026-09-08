#ifndef BNO085_DRIVER__NMEA_PARSER_HPP_
#define BNO085_DRIVER__NMEA_PARSER_HPP_

#include <string>

namespace bno085_driver
{

bool parseHdm(
  const std::string & sentence,
  double & heading_deg);

}  // namespace bno085_driver

#endif  // BNO085_DRIVER__NMEA_PARSER_HPP_
