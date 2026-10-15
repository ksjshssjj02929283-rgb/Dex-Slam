/*
 * pbstream 地图数据版本。
 *
 * SerializationHeader.data_version 记录一张 pbstream「存了哪些数据」，离线工具
 * 据此决定怎么重建，而不是逐节点猜。版本历史：
 *
 *   （空）  0.3.0 之前保存的旧图。节点只有滤波后点云（字段 3），没有原始
 *           range data，高亮点云也未写入。离线重建只能由滤波点云合成 range
 *           data：无 misses，无回波光束扫出的 free space 会丢失。
 *   0.3.0   节点持久化去畸变原始 range data（字段 11）与高亮点云（字段 10）。
 *           离线重建与建图输出逐 cell 一致。
 *
 * 版本只能「继承或降低」，不能凭空升级：加载一张旧图后再保存（定位回写、续建）
 * 仍是旧图，因为那些节点确实没有原始数据。
 */
#ifndef CARTOGRAPHER_IO_PBSTREAM_VERSION_H_
#define CARTOGRAPHER_IO_PBSTREAM_VERSION_H_

#include <string>

#include "cartographer/mapping/proto/serialization.pb.h"

namespace cartographer {
namespace io {

// 本程序写出的地图数据版本。
constexpr char kPbstreamDataVersion[] = "0.3.0";
// 从这个版本起，节点带原始 range data 与高亮点云。
constexpr char kPbstreamRangeDataSinceVersion[] = "0.3.0";

// 比较两个数据版本（"主.次.修订"）：a<b 返回负数，相等返回 0，a>b 返回正数。
// 空串（旧图）小于任何版本。
int ComparePbstreamDataVersion(const std::string& a, const std::string& b);

// 两者中较低的版本。
std::string MinPbstreamDataVersion(const std::string& a, const std::string& b);

// 该版本的节点是否带原始 range data（可无损离线重建）。
bool PbstreamDataVersionHasRangeData(const std::string& data_version);
bool PbstreamHasRangeData(const mapping::proto::SerializationHeader& header);

// 用于日志 / 诊断输出：空串显示为 "legacy (<0.3.0)"。
std::string PbstreamDataVersionLabel(const std::string& data_version);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_PBSTREAM_VERSION_H_
