#include "bno085_driver/nmea_parser.hpp"

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>

namespace bno085_driver
{

namespace
{

int hexValue(char character)
{
  if ((character >= '0') && (character <= '9')) {
    return character - '0';
  }

  if ((character >= 'A') && (character <= 'F')) {
    return character - 'A' + 10;
  }

  if ((character >= 'a') && (character <= 'f')) {
    return character - 'a' + 10;
  }

  return -1;
}

}  // namespace


bool parseHdm(
  const std::string & sentence,
  double & heading_deg)
{
  if (sentence.empty() || (sentence.front() != '$')) {
    return false;
  }

  const std::size_t checksum_position =
    sentence.find('*');

  if ((checksum_position == std::string::npos) ||
    ((checksum_position + 2U) >= sentence.size()))
  {
    return false;
  }

  uint8_t calculated_checksum = 0U;

  for (std::size_t i = 1U; i < checksum_position; ++i) {
    calculated_checksum ^=
      static_cast<uint8_t>(sentence[i]);
  }

  const int checksum_high =
    hexValue(sentence[checksum_position + 1U]);

  const int checksum_low =
    hexValue(sentence[checksum_position + 2U]);

  if ((checksum_high < 0) || (checksum_low < 0)) {
    return false;
  }

  const uint8_t received_checksum =
    static_cast<uint8_t>(
    (checksum_high << 4) | checksum_low);

  if (calculated_checksum != received_checksum) {
    return false;
  }

  const std::string body =
    sentence.substr(
    1U,
    checksum_position - 1U);

  constexpr char prefix[] = "HCHDM,";

  if (body.rfind(prefix, 0U) != 0U) {
    return false;
  }

  const std::size_t heading_start =
    sizeof(prefix) - 1U;

  const std::size_t heading_end =
    body.find(',', heading_start);

  if (heading_end == std::string::npos) {
    return false;
  }

  if (body.substr(heading_end + 1U) != "M") {
    return false;
  }

  const std::string heading_text =
    body.substr(
    heading_start,
    heading_end - heading_start);

  char * end_ptr = nullptr;

  errno = 0;

  const double parsed_heading =
    std::strtod(
    heading_text.c_str(),
    &end_ptr);

  if ((errno != 0) ||
    (end_ptr == heading_text.c_str()) ||
    (*end_ptr != '\0') ||
    (!std::isfinite(parsed_heading)) ||
    (parsed_heading < 0.0) ||
    (parsed_heading >= 360.0))
  {
    return false;
  }

  heading_deg = parsed_heading;

  return true;
}

}  // namespace bno085_driver
