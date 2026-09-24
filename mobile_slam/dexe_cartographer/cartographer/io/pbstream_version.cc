#include "cartographer/io/pbstream_version.h"

#include <array>
#include <cstdlib>

#include "glog/logging.h"

namespace cartographer {
namespace io {
namespace {

// "主.次.修订" → 三段整数；缺的段按 0。空串返回 {-1,-1,-1}，排在所有版本之前。
std::array<int, 3> ParseVersion(const std::string& version) {
  if (version.empty()) return {{-1, -1, -1}};
  std::array<int, 3> parts{{0, 0, 0}};
  size_t begin = 0;
  for (int i = 0; i < 3 && begin <= version.size(); ++i) {
    const size_t dot = version.find('.', begin);
    const std::string token = version.substr(
        begin, dot == std::string::npos ? std::string::npos : dot - begin);
    char* end = nullptr;
    const long value = std::strtol(token.c_str(), &end, 10);
    LOG_IF(WARNING, token.empty() || *end != '\0')
        << "Malformed pbstream data_version '" << version << "'";
    parts[i] = static_cast<int>(value);
    if (dot == std::string::npos) break;
    begin = dot + 1;
  }
  return parts;
}

}  // namespace

int ComparePbstreamDataVersion(const std::string& a, const std::string& b) {
  const auto pa = ParseVersion(a);
  const auto pb = ParseVersion(b);
  for (int i = 0; i < 3; ++i) {
    if (pa[i] != pb[i]) return pa[i] < pb[i] ? -1 : 1;
  }
  return 0;
}

std::string MinPbstreamDataVersion(const std::string& a, const std::string& b) {
  return ComparePbstreamDataVersion(a, b) <= 0 ? a : b;
}

bool PbstreamDataVersionHasRangeData(const std::string& data_version) {
  return ComparePbstreamDataVersion(data_version,
                                    kPbstreamRangeDataSinceVersion) >= 0;
}

bool PbstreamHasRangeData(const mapping::proto::SerializationHeader& header) {
  return PbstreamDataVersionHasRangeData(header.data_version());
}

std::string PbstreamDataVersionLabel(const std::string& data_version) {
  return data_version.empty() ? std::string("legacy (<") +
                                    kPbstreamRangeDataSinceVersion + ")"
                              : data_version;
}

}  // namespace io
}  // namespace cartographer
